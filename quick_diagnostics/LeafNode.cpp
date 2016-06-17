#include "LeafNode.h"

const QColor LeafNode::loColor = Qt::blue;
const QColor LeafNode::hiColor = Qt::red;
const QColor LeafNode::errorColor = Qt::yellow;

// NB: don't display any text in a leaf node. Instead, display a tooltip on mouseOver
LeafNode::LeafNode(GetData::Dirfile* dirfile, const char* fieldCode, double lo, double hi) : QLabel(), dirfile(dirfile), fieldCode(fieldCode), lo(lo), hi(hi) {
  setMinimumSize(5, 5); // allow the node to be very small

  isSelected = false;
  setMouseTracking(true); // if false, only receives mouseMoveEvent when mouse is pressed, see documentation

  // Start with a dirfile error b/c we haven't yet read the dirfile for this node
  errorList = new QList<QString>();
  errorList->append("Have not yet read dirfile");
}

void LeafNode::select() {
  isSelected = true;
  update();
}

void LeafNode::unselect() {
  isSelected = false;
  update();
}

QString LeafNode::getFieldCode() {
  return fieldCode;
}

double LeafNode::getCurrentValue() {
  return currentValue;
}

double LeafNode::getLoValue() {
  return lo;
}

double LeafNode::getHiValue() {
  return hi;
}

const QList<QString>& LeafNode::getErrorList() {
  return *errorList;
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

// Read the most recent data from the dirfile, and update status accordingly
void LeafNode::updateStatus(int frameNum) {

  // Reset the error list (those are errors from the last read)
  while (!errorList->empty()) {
    errorList->takeFirst();
  }

  double buffer[1];
  int numSamplesRead = dirfile->GetData(fieldCode, frameNum - 1, 0, 0, 1, GetData::Float64, (void*) buffer);

  // Check for dirfile errors
  if (dirfile->Error() != GD_E_OK) {
    QString e ("<i>(when reading)</i> ");
    e += dirfile->ErrorString();
    errorList->append(e);
  }

  // Expected to read 1 sample
  if (numSamplesRead != 1) {
    QString e("<i>(when reading)</i> num samples read: ");
    e += QString::number(numSamplesRead);
    e += ". Expected 1.";
    errorList->append(e);
  }

  // When done reading the field, flush to close the file descriptor until next read
  // to prevent having open too many file descriptors. Limit is ~1024, typically, I think.
  dirfile->Flush(fieldCode);
  if (dirfile->Error() != GD_E_OK) {
    QString e("<i>(when flushing)</i> ");
    e += dirfile->ErrorString();
    errorList->append(e);
  }

  // If no error, update the current value
  if (errorList->empty()) {
    currentValue = buffer[0];
  }   

  // Trigger a repaint event, since the status color may have changed
  update(); 
}

// Custom paint this node
void LeafNode::paintEvent(QPaintEvent* /*evt*/) {
  QRect geo = geometry();
  QPainter p(this);

  // Update the status color, and set the brush to that color
  QColor statusColor;
  if (errorList->empty()) {
    // Based on current val's proximity to lo and hi values, interpolate the status color between blue (lo), white (avg), and red (high)
    float avg = (lo + hi) / 2.0;
    if (currentValue > avg) {
      // if over hi, consider it hi
      interpolateColor(statusColor, fmin(currentValue, hi), avg, hi, Qt::white, hiColor);    
    } else {
      // if under lo, conside it lo
      interpolateColor(statusColor, fmax(currentValue, lo), lo, avg, loColor, Qt::white);  
    }
  } else {
    statusColor = errorColor;
  }
  p.setBrush(statusColor);
  p.drawRect(0, 0, geo.width(), geo.height());
  
  // If this node is selected, draw a border
  if (isSelected) {
    QPen pen(Qt::white, 3);
    p.setPen(pen);
    p.drawRect(0, 0, geo.width(), geo.height());
  }
}

// When the mouse moves in this widget, display the tooltip
void LeafNode::mouseMoveEvent(QMouseEvent* evt) {
  QToolTip::showText(evt->globalPos(), fieldCode);
}

// When the mouse leaves this widget, hide the tooltip
void LeafNode::leaveEvent(QEvent* /*evt*/) {
  QToolTip::hideText();
}

// Emit a clicked signal when this label is pressed
// NB: evt is not used
void LeafNode::mousePressEvent(QMouseEvent* /*evt*/) {
  emit clicked(this);
}
