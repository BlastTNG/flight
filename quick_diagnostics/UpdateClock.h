#ifndef UPDATECLOCK_H
#define UPDATECLOCK_H

#include <QtGui>
#include <time.h>

class UpdateClock : public QWidget {
  Q_OBJECT
public:
  UpdateClock() : QWidget() {

    setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    timerLabel = new QLabel();
    frameLabel = new QLabel();

    QGridLayout* layout = new QGridLayout();
    layout->addWidget(new QLabel("Time since new frame(s):"), 0, 0);
    layout->addWidget(timerLabel, 0, 1);
    layout->addWidget(new QLabel("Num frames on last read:"), 1, 0);
    layout->addWidget(frameLabel, 1, 1);
    setLayout(layout);

    QTimer* timer = new QTimer();
    QObject::connect(timer, SIGNAL(timeout()), this, SLOT(updateTime())); 
    timer->start(100);
    time(&startTime);
  }
  ~UpdateClock() {};
  
  // Tell the clock how many frames there were in the dirfile when it was last read
  void updateNumFrames(int numFrames) {

    // If the number of frames changed, update the label and recent the timer
    if (numFrames != lastNumFrames) {
      lastNumFrames = numFrames;
      frameLabel->setText(QString::number(numFrames));
      time(&startTime);
    }
  }
private:
  int lastNumFrames; // the number of frames on the last read
  time_t startTime;
  QLabel* timerLabel;
  QLabel* frameLabel;
private slots:
  void updateTime() {
    // Display the number of seconds since the timer was started
    time_t endTime;
    time(&endTime);
    double secsElapsed = difftime(endTime, startTime);
    QString s;
    timerLabel->setText(s.sprintf("%5.2fs", secsElapsed));
  }
};

#endif
