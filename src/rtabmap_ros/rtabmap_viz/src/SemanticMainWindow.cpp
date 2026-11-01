#include "rtabmap_viz/SemanticMainWindow.h"
#include "rtabmap_viz/SemanticLabels.h"
#include <rtabmap/gui/CloudViewer.h>
#include <QAction>
#include <QFileInfo>
#include <QToolBar>
#include <QTimer>
#include <vtkBillboardTextActor3D.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkTextProperty.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rtabmap_viz {

SemanticMainWindow::SemanticMainWindow(rtabmap::PreferencesDialog * preferences,
    const std::string & mapKey, double textScale, double labelOffset, bool showLabelDetails) :
    MainWindow(preferences), mapKey_(mapKey), textScale_(textScale), labelOffset_(labelOffset),
    showLabelDetails_(showLabelDetails)
{
  for (const auto path : {"/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
                          "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"}) {
    if (QFileInfo::exists(path)) { fontFile_ = path; break; }
  }
  if (fontFile_.empty()) throw std::runtime_error("Chinese labels require fonts-droid-fallback or fonts-noto-cjk");
  auto toolbar = addToolBar(tr("语义物体"));
  toolbar->setObjectName("semanticObjectsToolbar");
  showSemantics_ = toolbar->addAction(tr("物体类别"));
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

vtkRenderer * SemanticMainWindow::textRenderer() const
{
  auto renderers = cloudViewer()->renderWindow()->GetRenderers();
  renderers->InitTraversal();
  while (auto renderer = renderers->GetNextItem()) {
    if (renderer->GetLayer() == 3) return renderer; // CloudViewer's foreground text layer.
  }
  return nullptr;
}

void SemanticMainWindow::removeSemanticText(const std::string & id)
{
  const auto found = textActors_.find(id);
  if (found != textActors_.end()) {
    if (auto renderer = textRenderer()) renderer->RemoveViewProp(found->second);
    textActors_.erase(found);
  }
  cloudViewer()->removeText(id);
}

void SemanticMainWindow::clearSemantics()
{
  const bool changed = !rendered_.empty();
  for (const auto & id : rendered_) {
    removeSemanticText(id);
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
  auto renderer = textRenderer();
  if (!renderer) return;
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
    const auto label = semanticDisplayLabel(object.label, object.object_id,
                                           object.confidence, showLabelDetails_);
    const std::string cacheText = label + (object.dynamic ? "|dynamic" : "|static");
    const auto previous = drawingCache_.find(id);
    const auto existingText = textActors_.find(id);
    if (previous != drawingCache_.end() && previous->second.second == cacheText &&
        (previous->second.first-position).squaredNorm() < 1e-8 &&
        existingText != textActors_.end() && renderer->HasViewProp(existingText->second) &&
        cloudViewer()->getAddedSpheres().count(id+"_point"))
      continue;
    drawingCache_[id] = std::make_pair(position, cacheText);
    changed = true;
    // PCL addText3D uses ASCII-only vtkVectorText. A billboard supports UTF-8/CJK fonts.
    auto & actor = textActors_[id];
    if (!actor) actor = vtkSmartPointer<vtkBillboardTextActor3D>::New();
    actor->SetInput(label.c_str());
    actor->SetPosition(position.x(), position.y(), position.z() + labelOffset_);
    auto text = actor->GetTextProperty();
    text->SetFontFamily(VTK_FONT_FILE);
    text->SetFontFile(fontFile_.c_str());
    // Keep the existing scale control: the default 0.12 selects an 18-pixel label.
    text->SetFontSize(std::max(10, std::min(72, static_cast<int>(std::lround(textScale_ * 150)))));
    text->SetColor(color.redF(), color.greenF(), color.blueF());
    text->SetJustificationToCentered();
    text->BoldOff();
    if (!renderer->HasViewProp(actor)) renderer->AddViewProp(actor);
    cloudViewer()->addOrUpdateSphere(id + "_point",
      rtabmap::Transform(position.x(), position.y(), position.z(), 0, 0, 0), 0.035, color);
  }
  for (const auto & id : rendered_) {
    if (next.count(id) == 0) {
      removeSemanticText(id);
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
