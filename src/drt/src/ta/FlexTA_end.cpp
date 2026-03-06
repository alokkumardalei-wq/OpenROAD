// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#include <memory>
#include <utility>
#include <vector>

#include "db/obj/frFig.h"
#include "db/obj/frShape.h"
#include "distributed/drUpdate.h"
#include "frBaseTypes.h"
#include "ta/FlexTA.h"

namespace drt {

void FlexTAWorker::saveToGuides()
{
  for (auto& iroute : iroutes_) {
    for (auto& pin_fig : iroute->getFigs()) {
      if (pin_fig->typeId() == tacPathSeg) {
        std::unique_ptr<frPathSeg> path_seg = std::make_unique<frPathSeg>(
            *static_cast<taPathSeg*>(pin_fig.get()));
        if (save_updates_) {
          drUpdate update(drUpdate::ADD_GUIDE);
          update.setPathSeg(*path_seg);
          update.setIndexInOwner(iroute->getGuide()->getIndexInOwner());
          update.setNet(iroute->getGuide()->getNet());
          design_->addUpdate(update);
        }
        path_seg->addToNet(iroute->getGuide()->getNet());
        auto guide = iroute->getGuide();
        std::vector<std::unique_ptr<frConnFig>> tmp;
        tmp.push_back(std::move(path_seg));
        guide->setRoutes(tmp);
      }
      // modify upper/lower segs
      // upper/lower seg will have longest wirelength
    }
  }
}

void FlexTAWorker::end()
{
  saveToGuides();
}

}  // namespace drt
