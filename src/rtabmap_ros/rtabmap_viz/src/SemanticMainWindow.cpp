#include "rtabmap_viz/SemanticMainWindow.h"
#include <rtabmap/gui/CloudViewer.h>
#include <QAction>
#include <QToolBar>
#include <QTimer>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace rtabmap_viz {

SemanticMainWindow::SemanticMainWindow(rtabmap::PreferencesDialog * preferences,
    const std::string & mapKey, double textScale, double labelOffset) :
    MainWindow(preferences), mapKey_(mapKey), textScale_(textScale), labelOffset_(labelOffset)
{
  auto toolbar = addToolBar(tr("Semantic objects"));
  toolbar->setObjectName("semanticObjectsToolbar");
  showSemantics_ = toolbar->addAction(tr("Object labels"));
  showSemantics_->setCheckable(true);
  showSemantics_->setChecked(true);
  connect(showSemantics_, &QAction::toggled, this, &SemanticMainWindow::redrawSemantics);
  connect(this, &MainWindow::statsProcessed, this, &SemanticMainWindow::redrawSemantics);
  connect(this, &MainWindow::rtabmapEvent3DMapProcessed, this, &SemanticMainWindow::redrawSemantics);
  // Covers cache clearing and UI refresh paths which do not emit graph signals.
  auto timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &SemanticMainWindow::redrawSemantics);
  timer->start(200);
}

void SemanticMainWindow::clearSemantics()
{
  const bool changed = !rendered_.empty();
  for (const auto & id : rendered_) {
    cloudViewer()->removeText(id);
    cloudViewer()->removeSphere(id + "_point");
  }
  rendered_.clear();
  drawingCache_.clear();
  if (changed) cloudViewer()->refreshView();
}

void SemanticMainWindow::setSemanticMap(const rtabmap_semantic_msgs::msg::SemanticMap & map)
{
  if (!mapKey_.empty() && map.map_key != mapKey_) return;
  if (map.map_uuid != semantics_.map_uuid) clearSemantics();
  semantics_ = map;
  redrawSemantics();
}

void SemanticMainWindow::redrawSemantics()
{
  if (!showSemantics_->isChecked()) {
    if (rendered_.empty()) return;
    clearSemantics();
    return;
  }
  std::set<std::string> next;
  bool changed = false;
  for (const auto & object : semantics_.objects) {
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    double totalWeight = 0.0;
    for (const auto & observation : object.observations) {
      auto pose = currentPosesMap().find(observation.node_id);
      if (pose == currentPosesMap().end() || pose->second.isNull()) continue;
      auto signature = cachedSignatures().find(observation.node_id);
      if (signature != cachedSignatures().end() &&
          std::abs(signature->getStamp() - observation.node_stamp) > 1e-6) continue;
      const auto & point = observation.position;
      const double weight = observation.weight;
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
          !std::isfinite(weight) || weight <= 0) continue;
      const auto world = pose->second * rtabmap::Transform(point.x, point.y, point.z, 0, 0, 0);
      sum += weight * Eigen::Vector3d(world.x(), world.y(), world.z());
      totalWeight += weight;
    }
    if (totalWeight <= 0.0) continue;
    const Eigen::Vector3d position = sum / totalWeight;
    const auto id = "semantic_object_" + std::to_string(object.object_id);
    next.insert(id);
    // A deterministic class color, independent of std::hash implementation.
    unsigned int hash = 2166136261u;
    for (unsigned char c : object.label) hash = (hash ^ c) * 16777619u;
    QColor color = object.dynamic ? QColor(Qt::yellow) : QColor::fromHsv(hash % 360, 180, 255);
    std::ostringstream label;
    label << object.label << " #" << object.object_id << " "
          << std::fixed << std::setprecision(2) << object.confidence;
    const std::string cacheText = label.str() + (object.dynamic ? "|dynamic" : "|static");
    const auto previous = drawingCache_.find(id);
    if (previous != drawingCache_.end() && previous->second.second == cacheText &&
        (previous->second.first-position).squaredNorm() < 1e-8 &&
        cloudViewer()->getAddedTexts().count(id) && cloudViewer()->getAddedSpheres().count(id+"_point"))
      continue;
    drawingCache_[id] = std::make_pair(position, cacheText);
    changed = true;
    cloudViewer()->addOrUpdateText(id, label.str(),
      rtabmap::Transform(position.x(), position.y(), position.z() + labelOffset_, 0, 0, 0),
      textScale_, color);
    cloudViewer()->addOrUpdateSphere(id + "_point",
      rtabmap::Transform(position.x(), position.y(), position.z(), 0, 0, 0), 0.035, color);
  }
  for (const auto & id : rendered_) {
    if (next.count(id) == 0) {
      cloudViewer()->removeText(id);
      cloudViewer()->removeSphere(id + "_point");
      drawingCache_.erase(id);
      changed = true;
    }
  }
  rendered_ = std::move(next);
  // VTK 9 needs an explicit render; QWidget::update() only repaints its cached framebuffer.
  if (changed) cloudViewer()->refreshView();
}
}
