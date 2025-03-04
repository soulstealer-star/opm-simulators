/*
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
#include "config.h"

#include <flow/flow_ebos_blackoil_legacyassembly.hpp>

#include <opm/material/common/ResetLocale.hpp>
#include <opm/simulators/flow/SimulatorFullyImplicitBlackoilEbos.hpp>
#include <opm/simulators/flow/Main.hpp>

namespace Opm {

std::unique_ptr<FlowMainEbos<Properties::TTag::EclFlowProblem>>
flowEbosBlackoilMainInit(int argc, char** argv, bool outputCout, bool outputFiles)
{
    // we always want to use the default locale, and thus spare us the trouble
    // with incorrect locale settings.
    resetLocale();

    return std::make_unique<FlowMainEbos<Properties::TTag::EclFlowProblem>>(
        argc, argv, outputCout, outputFiles);
}

// ----------------- Main program -----------------
int flowEbosBlackoilMain(int argc, char** argv, bool outputCout, bool outputFiles)
{
    auto mainfunc = flowEbosBlackoilMainInit(argc, argv, outputCout, outputFiles);
    return mainfunc->execute();
}

int flowEbosBlackoilMainStandalone(int argc, char** argv)
{
    using TypeTag = Properties::TTag::EclFlowProblem;
    auto mainObject = std::make_unique<Opm::Main>(argc, argv);
    auto ret = mainObject->runStatic<TypeTag>();
    // Destruct mainObject as the destructor calls MPI_Finalize!
    mainObject.reset();
    return ret;
}

}
