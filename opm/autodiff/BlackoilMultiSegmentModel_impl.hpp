/*
  Copyright 2013, 2015 SINTEF ICT, Applied Mathematics.

  This file is part of the Open Porous Media project (OPM).

  OPM is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  OPM is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with OPM.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef OPM_BLACKOIMULTISEGMENTLMODEL_IMPL_HEADER_INCLUDED
#define OPM_BLACKOIMULTISEGMENTLMODEL_IMPL_HEADER_INCLUDED

#include <opm/autodiff/BlackoilMultiSegmentModel.hpp>

#include <opm/autodiff/AutoDiffBlock.hpp>
#include <opm/autodiff/AutoDiffHelpers.hpp>
#include <opm/autodiff/GridHelpers.hpp>
#include <opm/autodiff/BlackoilPropsAdInterface.hpp>
#include <opm/autodiff/GeoProps.hpp>
#include <opm/autodiff/WellDensitySegmented.hpp>
#include <opm/autodiff/VFPProperties.hpp>
#include <opm/autodiff/VFPProdProperties.hpp>
#include <opm/autodiff/VFPInjProperties.hpp>

#include <opm/core/grid.h>
#include <opm/core/linalg/LinearSolverInterface.hpp>
#include <opm/core/linalg/ParallelIstlInformation.hpp>
#include <opm/core/props/rock/RockCompressibility.hpp>
#include <opm/common/ErrorMacros.hpp>
#include <opm/common/Exceptions.hpp>
#include <opm/core/utility/Units.hpp>
#include <opm/core/well_controls.h>
#include <opm/core/utility/parameters/ParameterGroup.hpp>
#include <opm/parser/eclipse/EclipseState/EclipseState.hpp>

#include <cassert>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <limits>
#include <vector>
//#include <fstream>


namespace Opm {


    template <class Grid>
    BlackoilMultiSegmentModel<Grid>::
    BlackoilMultiSegmentModel(const typename Base::ModelParameters&  param,
                  const Grid&                     grid ,
                  const BlackoilPropsAdInterface& fluid,
                  const DerivedGeology&           geo  ,
                  const RockCompressibility*      rock_comp_props,
                  const Wells*                    wells_arg,
                  const NewtonIterationBlackoilInterface&    linsolver,
                  Opm::EclipseStateConstPtr eclState,
                  const bool has_disgas,
                  const bool has_vapoil,
                  const bool terminal_output,
                  const std::vector<WellMultiSegmentConstPtr>& wells_multisegment)
        : Base(param, grid, fluid, geo, rock_comp_props, wells_arg, linsolver,
               eclState, has_disgas, has_vapoil, terminal_output)
        , ms_wells_(wells_multisegment, fluid.numPhases())
    {
    }





    template <class Grid>
    void
    BlackoilMultiSegmentModel<Grid>::
    prepareStep(const double dt,
                ReservoirState& reservoir_state,
                WellState& well_state)
    {
        pvdt_ = geo_.poreVolume() / dt;
        if (active_[Gas]) {
            updatePrimalVariableFromState(reservoir_state);
        }

        msWells().topWellSegments() = well_state.topSegmentLoc();

        const int nw = wellsMultiSegment().size();

        if ( !msWellOps().has_multisegment_wells ) {
            msWells().segVDt() = V::Zero(nw);
            return;
        }

        const int nseg_total = well_state.numSegments();
        std::vector<double> segment_volume;
        segment_volume.reserve(nseg_total);
        for (int w = 0; w < nw; ++w) {
            WellMultiSegmentConstPtr well = wellsMultiSegment()[w];
            const std::vector<double>& segment_volume_well = well->segmentVolume();
            segment_volume.insert(segment_volume.end(), segment_volume_well.begin(), segment_volume_well.end());
        }
        assert(int(segment_volume.size()) == nseg_total);
        msWells().segVDt() = Eigen::Map<V>(segment_volume.data(), nseg_total) / dt;
    }





    template <class Grid>
    int
    BlackoilMultiSegmentModel<Grid>::numWellVars() const
    {
        // For each segment, we have a pressure variable, and one flux per phase.
        const int nseg = msWellOps().p2s.rows();
        return (numPhases() + 1) * nseg;
    }





    template <class Grid>
    void
    BlackoilMultiSegmentModel<Grid>::makeConstantState(SolutionState& state) const
    {
        Base::makeConstantState(state);
        state.segp  = ADB::constant(state.segp.value());
        state.segqs = ADB::constant(state.segqs.value());
    }





    template <class Grid>
    void
    BlackoilMultiSegmentModel<Grid>::variableWellStateInitials(const WellState& xw, std::vector<V>& vars0) const
    {
        // Initial well rates
        if ( wellsMultiSegment().size() > 0 )
        {
            // Need to reshuffle well segment rates, from phase running fastest
            const int nseg = xw.numSegments();
            const int np = xw.numPhases();

            // The transpose() below switches the ordering of the segment rates
            const DataBlock segrates = Eigen::Map<const DataBlock>(& xw.segPhaseRates()[0], nseg, np).transpose();
            // segment phase rates in surface volume
            const V segqs = Eigen::Map<const V>(segrates.data(), nseg * np);
            vars0.push_back(segqs);

            // for the pressure of the segments
            const V segp = Eigen::Map<const V>(& xw.segPress()[0], xw.segPress().size());
            vars0.push_back(segp);
        }
        else
        {
            // push null sates for segqs and segp
            vars0.push_back(V());
            vars0.push_back(V());
        }
    }





    template <class Grid>
    void
    BlackoilMultiSegmentModel<Grid>::variableStateExtractWellsVars(const std::vector<int>& indices,
                                                                   std::vector<ADB>& vars,
                                                                   SolutionState& state) const
    {
        // TODO: using the original Qs for the segment rates for now, to be fixed eventually.
        // TODO: using the original Bhp for the segment pressures for now, to be fixed eventually.

        // segment phase rates in surface volume
        state.segqs = std::move(vars[indices[Qs]]);

        // segment pressures
        state.segp = std::move(vars[indices[Bhp]]);

        // The qs and bhp are no longer primary variables, but could
        // still be used in computations. They are identical to the
        // pressures and flows of the top segments.
        const int np = numPhases();
        const int ns = state.segp.size();
        const int nw = msWells().topWellSegments().size();
        state.qs = ADB::constant(ADB::V::Zero(np*nw));
        for (int phase = 0; phase < np; ++phase) {
            // Extract segment fluxes for this phase (ns consecutive elements).
            ADB segqs_phase = subset(state.segqs, Span(ns, 1, ns*phase));
            // Extract top segment fluxes (= well fluxes)
            ADB wellqs_phase = subset(segqs_phase, msWells().topWellSegments());
            // Expand to full size of qs (which contains all phases) and add.
            state.qs += superset(wellqs_phase, Span(nw, 1, nw*phase), nw*np);
        }
        state.bhp = subset(state.segp, msWells().topWellSegments());
    }




    // TODO: This is just a preliminary version, remains to be improved later when we decide a better way
    // TODO: to intergrate the usual wells and multi-segment wells.
    template <class Grid>
    void BlackoilMultiSegmentModel<Grid>::computeWellConnectionPressures(const SolutionState& state,
                                                                         const WellState& xw)
    {
        if( ! wellsActive() ) return ;

        using namespace Opm::AutoDiffGrid;
        // 1. Compute properties required by computeConnectionPressureDelta().
        //    Note that some of the complexity of this part is due to the function
        //    taking std::vector<double> arguments, and not Eigen objects.
        const int nperf_total = xw.numPerforations();
        const int nw = xw.numWells();

        const std::vector<int>& well_cells = msWellOps().well_cells;

        stdWells().wellPerforationDensities() = V::Zero(nperf_total);

        const V perf_press = Eigen::Map<const V>(xw.perfPress().data(), nperf_total);

        V avg_press = perf_press * 0.0;

        // for the non-segmented/regular wells, calculated the average pressures.
        // If it is the top perforation, then average with the bhp().
        // If it is not the top perforation, then average with the perforation above it().
        int start_segment = 0;
        for (int w = 0; w < nw; ++w) {
            const int nseg = wellsMultiSegment()[w]->numberOfSegments();
            if (wellsMultiSegment()[w]->isMultiSegmented()) {
                // maybe we should give some reasonable values to prevent the following calculations fail
                start_segment += nseg;
                continue;
            }

            std::string well_name(wellsMultiSegment()[w]->name());
            typedef typename WellStateMultiSegment::SegmentedWellMapType::const_iterator const_iterator;
            const_iterator it_well = xw.segmentedWellMap().find(well_name);
            assert(it_well != xw.segmentedWellMap().end());

            const int start_perforation = (*it_well).second.start_perforation;
            const int end_perforation = start_perforation + (*it_well).second.number_of_perforations;
            for (int perf = start_perforation; perf < end_perforation; ++perf) {
                const double p_above = perf == start_perforation ? state.segp.value()[start_segment] : perf_press[perf - 1];
                const double p_avg = (perf_press[perf] + p_above)/2;
                avg_press[perf] = p_avg;
            }
            start_segment += nseg;
        }
        assert(start_segment == xw.numSegments());

        // Use cell values for the temperature as the wells don't knows its temperature yet.
        const ADB perf_temp = subset(state.temperature, well_cells);

        // Compute b, rsmax, rvmax values for perforations.
        // Evaluate the properties using average well block pressures
        // and cell values for rs, rv, phase condition and temperature.
        const ADB avg_press_ad = ADB::constant(avg_press);
        std::vector<PhasePresence> perf_cond(nperf_total);
        const std::vector<PhasePresence>& pc = phaseCondition();
        for (int perf = 0; perf < nperf_total; ++perf) {
            perf_cond[perf] = pc[well_cells[perf]];
        }
        const PhaseUsage& pu = fluid_.phaseUsage();
        DataBlock b(nperf_total, pu.num_phases);
        std::vector<double> rsmax_perf(nperf_total, 0.0);
        std::vector<double> rvmax_perf(nperf_total, 0.0);
        if (pu.phase_used[BlackoilPhases::Aqua]) {
            const V bw = fluid_.bWat(avg_press_ad, perf_temp, well_cells).value();
            b.col(pu.phase_pos[BlackoilPhases::Aqua]) = bw;
        }
        assert(active_[Oil]);
        const V perf_so =  subset(state.saturation[pu.phase_pos[Oil]].value(), well_cells);
        if (pu.phase_used[BlackoilPhases::Liquid]) {
            const ADB perf_rs = subset(state.rs, well_cells);
            const V bo = fluid_.bOil(avg_press_ad, perf_temp, perf_rs, perf_cond, well_cells).value();
            b.col(pu.phase_pos[BlackoilPhases::Liquid]) = bo;
            const V rssat = fluidRsSat(avg_press, perf_so, well_cells);
            rsmax_perf.assign(rssat.data(), rssat.data() + nperf_total);
        }
        if (pu.phase_used[BlackoilPhases::Vapour]) {
            const ADB perf_rv = subset(state.rv, well_cells);
            const V bg = fluid_.bGas(avg_press_ad, perf_temp, perf_rv, perf_cond, well_cells).value();
            b.col(pu.phase_pos[BlackoilPhases::Vapour]) = bg;
            const V rvsat = fluidRvSat(avg_press, perf_so, well_cells);
            rvmax_perf.assign(rvsat.data(), rvsat.data() + nperf_total);
        }
        // b is row major, so can just copy data.
        std::vector<double> b_perf(b.data(), b.data() + nperf_total * pu.num_phases);
        // Extract well connection depths.
        const V depth = cellCentroidsZToEigen(grid_);
        const V perfcelldepth = subset(depth, well_cells);
        std::vector<double> perf_cell_depth(perfcelldepth.data(), perfcelldepth.data() + nperf_total);

        // Surface density.
        // The compute density segment wants the surface densities as
        // an np * number of wells cells array
        V rho = superset(fluid_.surfaceDensity(0 , well_cells), Span(nperf_total, pu.num_phases, 0), nperf_total * pu.num_phases);
        for (int phase = 1; phase < pu.num_phases; ++phase) {
            rho += superset(fluid_.surfaceDensity(phase , well_cells), Span(nperf_total, pu.num_phases, phase), nperf_total * pu.num_phases);
        }
        std::vector<double> surf_dens_perf(rho.data(), rho.data() + nperf_total * pu.num_phases);

        // Gravity
        double grav = detail::getGravity(geo_.gravity(), dimensions(grid_));

        // 2. Compute densities
        std::vector<double> cd =
                WellDensitySegmented::computeConnectionDensities(
                        wells(), xw, fluid_.phaseUsage(),
                        b_perf, rsmax_perf, rvmax_perf, surf_dens_perf);

        // 3. Compute pressure deltas
        std::vector<double> cdp =
                WellDensitySegmented::computeConnectionPressureDelta(
                        wells(), perf_cell_depth, cd, grav);

        // 4. Store the results
        stdWells().wellPerforationDensities() = Eigen::Map<const V>(cd.data(), nperf_total); // This one is not useful for segmented wells at all
        stdWells().wellPerforationPressureDiffs() = Eigen::Map<const V>(cdp.data(), nperf_total);

        if ( !msWellOps().has_multisegment_wells ) {
            msWells().wellPerforationCellDensities() = V::Zero(nperf_total);
            msWells().wellPerforationCellPressureDiffs() = V::Zero(nperf_total);
            return;
        }

        // compute the average of the fluid densites in the well blocks.
        // the average is weighted according to the fluid relative permeabilities.
        const std::vector<ADB> kr_adb = Base::computeRelPerm(state);
        size_t temp_size = kr_adb.size();
        std::vector<V> perf_kr;
        for(size_t i = 0; i < temp_size; ++i) {
            // const ADB kr_phase_adb = subset(kr_adb[i], well_cells);
            const V kr_phase = (subset(kr_adb[i], well_cells)).value();
            perf_kr.push_back(kr_phase);
        }


        // compute the averaged density for the well block
        // TODO: for the non-segmented wells, they should be set to zero
        // TODO: for the moment, they are still calculated, while not used later.
        for (int i = 0; i < nperf_total; ++i) {
            double sum_kr = 0.;
            int np = perf_kr.size(); // make sure it is 3
            for (int p = 0;  p < np; ++p) {
                sum_kr += perf_kr[p][i];
            }

            for (int p = 0; p < np; ++p) {
                perf_kr[p][i] /= sum_kr;
            }
        }

        V rho_avg_perf = V::Constant(nperf_total, 0.0);
        // TODO: make sure the order of the density and the order of the kr are the same.
        for (int phaseIdx = 0; phaseIdx < fluid_.numPhases(); ++phaseIdx) {
            const int canonicalPhaseIdx = canph_[phaseIdx];
            const ADB fluid_density = fluidDensity(canonicalPhaseIdx, rq_[phaseIdx].b, state.rs, state.rv);
            const V rho_perf = subset(fluid_density, well_cells).value();
            // TODO: phaseIdx or canonicalPhaseIdx ?
            rho_avg_perf += rho_perf * perf_kr[phaseIdx];
        }

        msWells().wellPerforationCellDensities() = Eigen::Map<const V>(rho_avg_perf.data(), nperf_total);

        // We should put this in a global class
        std::vector<double> perf_depth_vec;
        perf_depth_vec.reserve(nperf_total);
        for (int w = 0; w < nw; ++w) {
            WellMultiSegmentConstPtr well = wellsMultiSegment()[w];
            const std::vector<double>& perf_depth_well = well->perfDepth();
            perf_depth_vec.insert(perf_depth_vec.end(), perf_depth_well.begin(), perf_depth_well.end());
        }
        assert(int(perf_depth_vec.size()) == nperf_total);
        const V perf_depth = Eigen::Map<V>(perf_depth_vec.data(), nperf_total);

        const V perf_cell_depth_diffs = perf_depth - perfcelldepth;

        msWells().wellPerforationCellPressureDiffs() = grav * msWells().wellPerforationCellDensities() * perf_cell_depth_diffs;


        // Calculating the depth difference between segment nodes and perforations.
        // TODO: should be put somewhere else for better clarity later
        msWells().wellSegmentPerforationDepthDiffs() = V::Constant(nperf_total, -1e100);

        int start_perforation = 0;
        for (int w = 0; w < nw; ++w) {
            WellMultiSegmentConstPtr well = wellsMultiSegment()[w];
            const int nseg = well->numberOfSegments();
            const int nperf = well->numberOfPerforations();
            const std::vector<std::vector<int>>& segment_perforations = well->segmentPerforations();
            for (int s = 0; s < nseg; ++s) {
                const int nperf_seg = segment_perforations[s].size();
                const double segment_depth = well->segmentDepth()[s];
                for (int perf = 0; perf < nperf_seg; ++perf) {
                    const int perf_number = segment_perforations[s][perf] + start_perforation;
                    msWells().wellSegmentPerforationDepthDiffs()[perf_number] = segment_depth - perf_depth[perf_number];
                }
            }
            start_perforation += nperf;
        }
        assert(start_perforation == nperf_total);
    }







    template <class Grid>
    void
    BlackoilMultiSegmentModel<Grid>::
    assemble(const ReservoirState& reservoir_state,
             WellState& well_state,
             const bool initial_assembly)
    {
        using namespace Opm::AutoDiffGrid;

        // TODO: include VFP effect.
        // If we have VFP tables, we need the well connection
        // pressures for the "simple" hydrostatic correction
        // between well depth and vfp table depth.
        //  if (isVFPActive()) {
        //     SolutionState state = asImpl().variableState(reservoir_state, well_state);
        //     SolutionState state0 = state;
        //     asImpl().makeConstantState(state0);
        //     asImpl().computeWellConnectionPressures(state0, well_state);
        // }

        // Possibly switch well controls and updating well state to
        // get reasonable initial conditions for the wells
        asImpl().updateWellControls(well_state);

        // Create the primary variables.
        SolutionState state = asImpl().variableState(reservoir_state, well_state);

        if (initial_assembly) {
            // Create the (constant, derivativeless) initial state.
            SolutionState state0 = state;
            asImpl().makeConstantState(state0);
            // Compute initial accumulation contributions
            // and well connection pressures.
            asImpl().computeAccum(state0, 0);
            msWells().computeSegmentFluidProperties(state0, phaseCondition(), active_, fluid_, numPhases());
            const int np = numPhases();
            assert(np == int(msWells().segmentCompSurfVolumeInitial().size()));
            for (int phase = 0; phase < np; ++phase) {
                msWells().segmentCompSurfVolumeInitial()[phase] = msWells().segmentCompSurfVolumeCurrent()[phase].value();
            }
            asImpl().computeWellConnectionPressures(state0, well_state);
        }

        // OPM_AD_DISKVAL(state.pressure);
        // OPM_AD_DISKVAL(state.saturation[0]);
        // OPM_AD_DISKVAL(state.saturation[1]);
        // OPM_AD_DISKVAL(state.saturation[2]);
        // OPM_AD_DISKVAL(state.rs);
        // OPM_AD_DISKVAL(state.rv);
        // OPM_AD_DISKVAL(state.qs);
        // OPM_AD_DISKVAL(state.bhp);

        // -------- Mass balance equations --------
        asImpl().assembleMassBalanceEq(state);

        // -------- Well equations ----------

        if ( ! wellsActive() ) {
            return;
        }

        // asImpl().computeSegmentFluidProperties(state);
        msWells().computeSegmentFluidProperties(state, phaseCondition(), active_, fluid_, numPhases());

        // asImpl().computeSegmentPressuresDelta(state);
        const double gravity = detail::getGravity(geo_.gravity(), UgGridHelpers::dimensions(grid_));
        msWells().computeSegmentPressuresDelta(gravity);

        std::vector<ADB> mob_perfcells;
        std::vector<ADB> b_perfcells;
        asImpl().extractWellPerfProperties(state, mob_perfcells, b_perfcells);
        if (param_.solve_welleq_initially_ && initial_assembly) {
            // solve the well equations as a pre-processing step
            asImpl().solveWellEq(mob_perfcells, b_perfcells, state, well_state);
        }

        // the perforation flux here are different
        // it is related to the segment location
        V aliveWells;
        std::vector<ADB> cq_s;
        const int nw = wellsMultiSegment().size();
        const int np = numPhases();
        const DataBlock compi = Eigen::Map<const DataBlock>(wells().comp_frac, nw, np);
        const V perf_press_diffs = stdWells().wellPerforationPressureDiffs();
        msWells().computeWellFlux(state, fluid_.phaseUsage(), active_,
                                  perf_press_diffs, compi,
                                  mob_perfcells, b_perfcells, np, aliveWells, cq_s);
        asImpl().updatePerfPhaseRatesAndPressures(cq_s, state, well_state);
        msWells().addWellFluxEq(cq_s, state, np, residual_);
        asImpl().addWellContributionToMassBalanceEq(cq_s, state, well_state);
        asImpl().addWellControlEq(state, well_state, aliveWells);
    }





    template <class Grid>
    void BlackoilMultiSegmentModel<Grid>::updatePerfPhaseRatesAndPressures(const std::vector<ADB>& cq_s,
                                                                           const SolutionState& state,
                                                                           WellState& xw) const
    {
        // Update the perforation phase rates (used to calculate the pressure drop in the wellbore).
        const int np = numPhases();
        const int nw = wellsMultiSegment().size();
        const int nperf_total = xw.perfPress().size();

        V cq = superset(cq_s[0].value(), Span(nperf_total, np, 0), nperf_total * np);
        for (int phase = 1; phase < np; ++phase) {
            cq += superset(cq_s[phase].value(), Span(nperf_total, np, phase), nperf_total * np);
        }
        xw.perfPhaseRates().assign(cq.data(), cq.data() + nperf_total * np);

        // Update the perforation pressures for usual wells first to recover the resutls
        // without mutlti segment wells. For segment wells, it has not been decided if
        // we need th concept of preforation pressures
        xw.perfPress().resize(nperf_total, -1.e100);

        const V& cdp = stdWells().wellPerforationPressureDiffs();
        int start_segment = 0;
        int start_perforation = 0;
        for (int i = 0; i < nw; ++i) {
            WellMultiSegmentConstPtr well = wellsMultiSegment()[i];
            const int nperf = well->numberOfPerforations();
            const int nseg = well->numberOfSegments();
            if (well->isMultiSegmented()) {
                start_segment += nseg;
                start_perforation += nperf;
                continue;
            }
            const V cdp_well = subset(cdp, Span(nperf, 1, start_perforation));
            const ADB segp = subset(state.segp, Span(nseg, 1, start_segment));
            const V perfpressure = (well->wellOps().s2p * segp.value().matrix()).array() + cdp_well;
            std::copy(perfpressure.data(), perfpressure.data() + nperf, &xw.perfPress()[start_perforation]);

            start_segment += nseg;
            start_perforation += nperf;
        }
    }





    template <class Grid>
    void BlackoilMultiSegmentModel<Grid>::updateWellControls(WellState& xw) const
    {
        if( ! wellsActive() ) return ;

        std::string modestring[4] = { "BHP", "THP", "RESERVOIR_RATE", "SURFACE_RATE" };
        // Find, for each well, if any constraints are broken. If so,
        // switch control to first broken constraint.
        const int np = wellsMultiSegment()[0]->numberOfPhases();
        const int nw = wellsMultiSegment().size();
        for (int w = 0; w < nw; ++w) {
            const WellControls* wc = wellsMultiSegment()[w]->wellControls();
            // The current control in the well state overrides
            // the current control set in the Wells struct, which
            // is instead treated as a default.
            int current = xw.currentControls()[w];
            // Loop over all controls except the current one, and also
            // skip any RESERVOIR_RATE controls, since we cannot
            // handle those.
            const int nwc = well_controls_get_num(wc);
            int ctrl_index = 0;
            for (; ctrl_index < nwc; ++ctrl_index) {
                if (ctrl_index == current) {
                    // This is the currently used control, so it is
                    // used as an equation. So this is not used as an
                    // inequality constraint, and therefore skipped.
                    continue;
                }
                if (wellhelpers::constraintBroken(
                        xw.bhp(), xw.thp(), xw.wellRates(),
                        w, np, wellsMultiSegment()[w]->wellType(), wc, ctrl_index)) {
                    // ctrl_index will be the index of the broken constraint after the loop.
                    break;
                }
            }

            if (ctrl_index != nwc) {
                // Constraint number ctrl_index was broken, switch to it.
                if (terminal_output_)
                {
                    std::cout << "Switching control mode for well " << wellsMultiSegment()[w]->name()
                              << " from " << modestring[well_controls_iget_type(wc, current)]
                              << " to " << modestring[well_controls_iget_type(wc, ctrl_index)] << std::endl;
                }
                xw.currentControls()[w] = ctrl_index;
                current = xw.currentControls()[w];
            }

            // Get gravity for THP hydrostatic corrrection
            // const double gravity = detail::getGravity(geo_.gravity(), UgGridHelpers::dimensions(grid_));

            // Updating well state and primary variables.
            // Target values are used as initial conditions for BHP, THP, and SURFACE_RATE
            const double target = well_controls_iget_target(wc, current);
            const double* distr = well_controls_iget_distr(wc, current);
            switch (well_controls_iget_type(wc, current)) {
            case BHP:
                xw.bhp()[w] = target;
                xw.segPress()[xw.topSegmentLoc()[w]] = target;
                break;

            case THP: {
                OPM_THROW(std::runtime_error, "THP control is not implemented for multi-sgement wells yet!!");
            }

            case RESERVOIR_RATE:
                // No direct change to any observable quantity at
                // surface condition.  In this case, use existing
                // flow rates as initial conditions as reservoir
                // rate acts only in aggregate.
                break;

            case SURFACE_RATE:
                for (int phase = 0; phase < np; ++phase) {
                    if (distr[phase] > 0.0) {
                        xw.wellRates()[np * w + phase] = target * distr[phase];
                        // TODO: consider changing all (not just top) segment rates
                        // to make them consistent, it could possibly improve convergence.
                        xw.segPhaseRates()[np * xw.topSegmentLoc()[w] + phase] = target * distr[phase];
                    }
                }
                break;
            }

        }
    }





    template <class Grid>
    bool BlackoilMultiSegmentModel<Grid>::solveWellEq(const std::vector<ADB>& mob_perfcells,
                                                      const std::vector<ADB>& b_perfcells,
                                                      SolutionState& state,
                                                      WellState& well_state)
    {
        const bool converged = baseSolveWellEq(mob_perfcells, b_perfcells, state, well_state);

        if (converged) {
            // We must now update the state.segp and state.segqs members,
            // that the base version does not know about.
            const int np = numPhases();
            const int nseg_total =well_state.numSegments();
            {
                // We will set the segp primary variable to the new ones,
                // but we do not change the derivatives here.
                ADB::V new_segp = Eigen::Map<ADB::V>(well_state.segPress().data(), nseg_total);
                // Avoiding the copy below would require a value setter method
                // in AutoDiffBlock.
                std::vector<ADB::M> old_segp_derivs = state.segp.derivative();
                state.segp = ADB::function(std::move(new_segp), std::move(old_segp_derivs));
            }
            {
                // Need to reshuffle well rates, from phase running fastest
                // to wells running fastest.
                // The transpose() below switches the ordering.
                const DataBlock segrates = Eigen::Map<const DataBlock>(well_state.segPhaseRates().data(), nseg_total, np).transpose();
                ADB::V new_segqs = Eigen::Map<const V>(segrates.data(), nseg_total * np);
                std::vector<ADB::M> old_segqs_derivs = state.segqs.derivative();
                state.segqs = ADB::function(std::move(new_segqs), std::move(old_segqs_derivs));
            }

            // This is also called by the base version, but since we have updated
            // state.segp we must call it again.
            asImpl().computeWellConnectionPressures(state, well_state);
        }

        return converged;
    }





    template <class Grid>
    void BlackoilMultiSegmentModel<Grid>::addWellControlEq(const SolutionState& state,
                                                           const WellState& xw,
                                                           const V& aliveWells)
    {
        // the name of the function is a a little misleading.
        // Basically it is the function for the pressure equation.
        // And also, it work as the control equation when it is the segment
        if( wellsMultiSegment().empty() ) return;

        const int np = numPhases();
        const int nw = wellsMultiSegment().size();
        const int nseg_total = xw.numSegments();

        ADB aqua   = ADB::constant(ADB::V::Zero(nseg_total));
        ADB liquid = ADB::constant(ADB::V::Zero(nseg_total));
        ADB vapour = ADB::constant(ADB::V::Zero(nseg_total));

        if (active_[Water]) {
            aqua += subset(state.segqs, Span(nseg_total, 1, BlackoilPhases::Aqua * nseg_total));
        }
        if (active_[Oil]) {
            liquid += subset(state.segqs, Span(nseg_total, 1, BlackoilPhases::Liquid * nseg_total));
        }
        if (active_[Gas]) {
            vapour += subset(state.segqs, Span(nseg_total, 1, BlackoilPhases::Vapour * nseg_total));
        }

        // THP control is not implemented for the moment.

        // Hydrostatic correction variables
        ADB::V rho_v = ADB::V::Zero(nw);
        ADB::V vfp_ref_depth_v = ADB::V::Zero(nw);

        // Target vars
        ADB::V bhp_targets  = ADB::V::Zero(nw);
        ADB::V rate_targets = ADB::V::Zero(nw);
        Eigen::SparseMatrix<double>  rate_distr(nw, np*nw);

        // Selection variables
        // well selectors
        std::vector<int> bhp_well_elems;
        std::vector<int> rate_well_elems;
        // segment selectors
        std::vector<int> bhp_top_elems;
        std::vector<int> rate_top_elems;
        std::vector<int> rate_top_phase_elems;
        std::vector<int> others_elems;

        //Run through all wells to calculate BHP/RATE targets
        //and gather info about current control
        int start_segment = 0;
        for (int w = 0; w < nw; ++w) {
            const struct WellControls* wc = wellsMultiSegment()[w]->wellControls();

            // The current control in the well state overrides
            // the current control set in the Wells struct, which
            // is instead treated as a default.
            const int current = xw.currentControls()[w];

            const int nseg = wellsMultiSegment()[w]->numberOfSegments();

            switch (well_controls_iget_type(wc, current)) {
            case BHP:
            {
                bhp_well_elems.push_back(w);
                bhp_top_elems.push_back(start_segment);
                bhp_targets(w)  = well_controls_iget_target(wc, current);
                rate_targets(w) = -1e100;
                for (int p = 0; p < np; ++p) {
                    rate_top_phase_elems.push_back(np * start_segment + p);
                }
            }
            break;

            case THP:
            {
                OPM_THROW(std::runtime_error, "THP control is not implemented for multi-sgement wells yet!!");
            }
            break;

            case RESERVOIR_RATE: // Intentional fall-through
            case SURFACE_RATE:
            {
                rate_well_elems.push_back(w);
                rate_top_elems.push_back(start_segment);
                for (int p = 0; p < np; ++p) {
                    rate_top_phase_elems.push_back(np * start_segment + p);
                }
                // RESERVOIR and SURFACE rates look the same, from a
                // high-level point of view, in the system of
                // simultaneous linear equations.

                const double* const distr =
                    well_controls_iget_distr(wc, current);

                for (int p = 0; p < np; ++p) {
                    rate_distr.insert(w, p*nw + w) = distr[p];
                }

                bhp_targets(w)  = -1.0e100;
                rate_targets(w) = well_controls_iget_target(wc, current);
            }
            break;

            }

            for (int i = 1; i < nseg; ++i) {
                others_elems.push_back(i + start_segment);
            }
            start_segment += nseg;
        }

        // for each segment: 1, if the segment is the top segment, then control equation
        //                   2, if the segment is not the top segment, then the pressure equation
        const ADB bhp_residual = subset(state.segp, bhp_top_elems) - subset(bhp_targets, bhp_well_elems);
        const ADB rate_residual = subset(rate_distr * subset(state.segqs, rate_top_phase_elems) - rate_targets, rate_well_elems);

        ADB others_residual = ADB::constant(V::Zero(nseg_total));

        if ( msWellOps().has_multisegment_wells ) {
            // Special handling for when we are called from solveWellEq().
            // TODO: restructure to eliminate need for special treatmemt.
            ADB wspd = (state.segp.numBlocks() == 2)
                ? wellhelpers::onlyWellDerivs(msWells().wellSegmentPressureDelta())
                : msWells().wellSegmentPressureDelta();

            others_residual = msWellOps().eliminate_topseg * (state.segp - msWellOps().s2s_outlet * state.segp + wspd);
        } else {
            others_residual = msWellOps().eliminate_topseg * (state.segp - msWellOps().s2s_outlet * state.segp);
        }

        //       all the control equations
        // TODO: can be optimized better
        ADB well_eq_topsegment = subset(superset(bhp_residual, bhp_top_elems, nseg_total) +
                                        superset(rate_residual, rate_top_elems, nseg_total), xw.topSegmentLoc());

        // For wells that are dead (not flowing), and therefore not communicating
        // with the reservoir, we set the equation to be equal to the well's total
        // flow. This will be a solution only if the target rate is also zero.
        Eigen::SparseMatrix<double> rate_summer(nw, np*nw);
        for (int w = 0; w < nw; ++w) {
            for (int phase = 0; phase < np; ++phase) {
                rate_summer.insert(w, phase*nw + w) = 1.0;
            }
        }
        Selector<double> alive_selector(aliveWells, Selector<double>::NotEqualZero);
        // TODO: Here only handles the wells, or the top segments
        // should we also handle some non-alive non-top segments?
        // should we introduce the cocept of non-alive segments?
        // At the moment, we only handle the control equations
        well_eq_topsegment = alive_selector.select(well_eq_topsegment, rate_summer * subset(state.segqs, rate_top_phase_elems));

        /* residual_.well_eq = superset(bhp_residual, bhp_top_elems, nseg_total) +
                            superset(rate_residual, rate_top_elems, nseg_total) +
                            superset(others_residual, others_elems, nseg_total); */
        residual_.well_eq = superset(well_eq_topsegment, xw.topSegmentLoc(), nseg_total) +
                            others_residual;

    }





    template <class Grid>
    void
    BlackoilMultiSegmentModel<Grid>::updateWellState(const V& dwells,
                                                     WellState& well_state)
    {
        msWells().updateWellState(dwells, fluid_.numPhases(), dpMaxRel(), well_state);
    }





        /// added to fixing the flow_multisegment running
    template <class Grid>
    bool
    BlackoilMultiSegmentModel<Grid>::baseSolveWellEq(const std::vector<ADB>& mob_perfcells,
                                                     const std::vector<ADB>& b_perfcells,
                                                     SolutionState& state,
                                                     WellState& well_state) {
        V aliveWells;
        const int np = wells().number_of_phases;
        std::vector<ADB> cq_s(np, ADB::null());
        std::vector<int> indices = stdWells().variableWellStateIndices();
        SolutionState state0 = state;
        WellState well_state0 = well_state;
        makeConstantState(state0);

        std::vector<ADB> mob_perfcells_const(np, ADB::null());
        std::vector<ADB> b_perfcells_const(np, ADB::null());

        if ( Base::localWellsActive() ){
            // If there are non well in the sudomain of the process
            // thene mob_perfcells_const and b_perfcells_const would be empty
            for (int phase = 0; phase < np; ++phase) {
                mob_perfcells_const[phase] = ADB::constant(mob_perfcells[phase].value());
                b_perfcells_const[phase] = ADB::constant(b_perfcells[phase].value());
            }
        }

        int it  = 0;
        bool converged;
        do {
            // bhp and Q for the wells
            std::vector<V> vars0;
            vars0.reserve(2);
            variableWellStateInitials(well_state, vars0);
            std::vector<ADB> vars = ADB::variables(vars0);

            SolutionState wellSolutionState = state0;
            variableStateExtractWellsVars(indices, vars, wellSolutionState);

            const int nw = wellsMultiSegment().size();
            const DataBlock compi = Eigen::Map<const DataBlock>(wells().comp_frac, nw, np);
            const V perf_press_diffs = stdWells().wellPerforationPressureDiffs();
            msWells().computeWellFlux(wellSolutionState, fluid_.phaseUsage(), active_,
                                      perf_press_diffs, compi,
                                      mob_perfcells_const, b_perfcells_const, np, aliveWells, cq_s);

            updatePerfPhaseRatesAndPressures(cq_s, wellSolutionState, well_state);
            msWells().addWellFluxEq(cq_s, wellSolutionState, np, residual_);
            addWellControlEq(wellSolutionState, well_state, aliveWells);
            converged = Base::getWellConvergence(it);

            if (converged) {
                break;
            }

            ++it;
            if( Base::localWellsActive() )
            {
                std::vector<ADB> eqs;
                eqs.reserve(2);
                eqs.push_back(residual_.well_flux_eq);
                eqs.push_back(residual_.well_eq);
                ADB total_residual = vertcatCollapseJacs(eqs);
                const std::vector<M>& Jn = total_residual.derivative();
                typedef Eigen::SparseMatrix<double> Sp;
                Sp Jn0;
                Jn[0].toSparse(Jn0);
                const Eigen::SparseLU< Sp > solver(Jn0);
                ADB::V total_residual_v = total_residual.value();
                const Eigen::VectorXd& dx = solver.solve(total_residual_v.matrix());
                assert(dx.size() == total_residual_v.size());
                asImpl().updateWellState(dx.array(), well_state);
                updateWellControls(well_state);
            }
        } while (it < 15);

        if (converged) {
            if ( terminal_output_ ) {
                std::cout << "well converged iter: " << it << std::endl;
            }
            const int nw = wells().number_of_wells;
            {
                // We will set the bhp primary variable to the new ones,
                // but we do not change the derivatives here.
                ADB::V new_bhp = Eigen::Map<ADB::V>(well_state.bhp().data(), nw);
                // Avoiding the copy below would require a value setter method
                // in AutoDiffBlock.
                std::vector<ADB::M> old_derivs = state.bhp.derivative();
                state.bhp = ADB::function(std::move(new_bhp), std::move(old_derivs));
            }
            {
                // Need to reshuffle well rates, from phase running fastest
                // to wells running fastest.
                // The transpose() below switches the ordering.
                const DataBlock wrates = Eigen::Map<const DataBlock>(well_state.wellRates().data(), nw, np).transpose();
                ADB::V new_qs = Eigen::Map<const V>(wrates.data(), nw*np);
                std::vector<ADB::M> old_derivs = state.qs.derivative();
                state.qs = ADB::function(std::move(new_qs), std::move(old_derivs));
            }
            computeWellConnectionPressures(state, well_state);
        }

        if (!converged) {
            well_state = well_state0;
        }

        return converged;
    }





    template <class Grid>
    std::vector<V>
    BlackoilMultiSegmentModel<Grid>::
    variableStateInitials(const ReservoirState& x,
                          const WellState&     xw) const
    {
        assert(active_[ Oil ]);

        const int np = x.numPhases();

        std::vector<V> vars0;
        // p, Sw and Rs, Rv or Sg is used as primary depending on solution conditions
        // and bhp and Q for the wells
        vars0.reserve(np + 1);
        variableReservoirStateInitials(x, vars0);
        variableWellStateInitials(xw, vars0);
        return vars0;
    }

} // namespace Opm

#endif // OPM_BLACKOILMODELBASE_IMPL_HEADER_INCLUDED
