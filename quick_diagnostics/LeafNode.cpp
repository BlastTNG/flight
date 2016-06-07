#include "LeafNode.h"

const QColor LeafNode::loColor = Qt::blue;
const QColor LeafNode::hiColor = Qt::red;

// NB: don't display any text in a leaf node. Instead, display a tooltip on mouseOver
LeafNode::LeafNode(GetData::Dirfile* dirfile, const char* fieldCode, double lo, double hi) : StatusNode(""), dirfile(dirfile), fieldCode(fieldCode), lo(lo), hi(hi) {
  isSelected = false;
  setMouseTracking(true); // if false, only receives mouseMoveEvent when mouse is pressed, see documentation
}

void LeafNode::select() {
  isSelected = true;
  update();
}

void LeafNode::unselect() {
  isSelected = false;
  update();
}

// Get val mapped from range currentLo <-> currentHi to the range newLo <-> newHi
float map(float val, float currentLo, float currentHi, float newLo, float newHi) {
  // Normalize the ranges (make their origin 0)
  float h1 = currentHi - currentLo;
  float normVal = val - currentLo;
  float h2 = newHi - newLo;
  float normResult = h2 * normVal / h1; // convert between normalized ranges
  return normResult + newLo; // un-normalize the range
}

// Interpolate color result between color aC and bC, based on val's position in the range from lo to hi.
void interpolateColor(QColor& result, float val, float lo, float hi, QColor aC, QColor bC) {
  int r = (int) map(val, lo, hi, aC.red(), bC.red());
  int g = (int) map(val, lo, hi, aC.green(), bC.green());
  int b = (int) map(val, lo, hi, aC.blue(), bC.blue());
  result = QColor::fromRgb(r, g, b);
}

// TODO: read the most recent sample from dirfile, update color accordingly
void LeafNode::updateStatus() {

  float val = rand() / (float) RAND_MAX;

  // If val > avg, interpolate between white and hiColor. If val <= avg, interpolate between white and loColor
  // Don't just interpolate between loColor and hiColor b/c it becomes difficult to judge the sensor value from the color
  float avg = (lo + hi) / 2;
  QString style;
  if (val > avg) {
    interpolateColor(statusColor, val, avg, hi, Qt::white, hiColor);    
  } else {
    interpolateColor(statusColor, val, lo, avg, loColor, Qt::white);  
  }
  update(); // triggers a repaint event
}

void LeafNode::paintEvent(QPaintEvent* evt) {
  QRect geo = geometry();
  QPainter p(this);
  p.setBrush(statusColor);
  p.drawRect(0, 0, geo.width(), geo.height());
  
  // If this node is selected, draw a border
  if (isSelected) {
    QPen pen(Qt::black, 5);
    p.setPen(pen);
    p.drawRect(0, 0, geo.width(), geo.height());
  }
}

// When the mouse moves in this widget, display the tooltip
void LeafNode::mouseMoveEvent(QMouseEvent* evt) {
  QToolTip::showText(evt->globalPos(), fieldCode);
}

// When the mouse leaves this widget, hide the tooltip
void LeafNode::leaveEvent(QEvent* evt) {
  QToolTip::hideText();
}

// Emit a clicked signal when this label is pressed
// NB: evt is not used
void LeafNode::mousePressEvent(QMouseEvent* /*evt*/) {
  emit clicked(fieldCode);
}
