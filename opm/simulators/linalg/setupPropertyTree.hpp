/*
  Copyright 2019 SINTEF Digital, Mathematics and Cybernetics.

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

#ifndef OPM_SETUPPROPERTYTREE_HEADER_INCLUDED
#define OPM_SETUPPROPERTYTREE_HEADER_INCLUDED

#include <opm/simulators/linalg/PropertyTree.hpp>

#include <string>

namespace Opm
{

struct FlowLinearSolverParameters;

PropertyTree setupPropertyTree(FlowLinearSolverParameters p,
                               bool linearSolverMaxIterSet,
                               bool linearSolverReductionSet);

PropertyTree setupCPRW(const std::string& conf, const FlowLinearSolverParameters& p);
PropertyTree setupCPR(const std::string& conf, const FlowLinearSolverParameters& p);
PropertyTree setupAMG(const std::string& conf, const FlowLinearSolverParameters& p);
PropertyTree setupILU(const std::string& conf, const FlowLinearSolverParameters& p);
PropertyTree setupUMFPack(const std::string& conf, const FlowLinearSolverParameters& p);

} // namespace Opm

#endif // OPM_SETUPPROPERTYTREE_HEADER_INCLUDED
