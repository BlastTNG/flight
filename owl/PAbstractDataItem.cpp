/* Owl: BLAST status GUI
 *
 * This file is part of Owl.
 *
 * Owl (originally "palantir") is copyright (C) 2002-2012 University of Toronto
 *
 * Owl is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Owl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Owl; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include "PAbstractDataItem.h"
#include "PMainWindow.h"
#include <QMouseEvent>
#include <QApplication>
#include <QDebug>
#include <QDrag>
#include <QMimeData>
#include <QProcess>
#include <QTemporaryFile>

//#include <QPlastiqueStyle>

#ifdef __APPLE__
#include <python2.6/Python.h>
#else
#include <python2.7/Python.h>   //you may need to change this
#endif

static int _delaysThisCycle = 0;

PAbstractDataItem::PAbstractDataItem(PBox* parent, QString caption) : QWidget(parent), _layout(new QHBoxLayout()),
    _caption(new QLabel(caption)), _captionStyle(PStyle::noStyle), _defaultDataStyle(PStyle::noStyle),
    _lastCapStyle(0), _data(new QLabel(tr("Loading"))), _serverDirty(-1), _neverGood(true), _sourceBad(false)
{
    connect(PStyleNotifier::me,SIGNAL(change()),this,SLOT(pstyleLogic()));
    setLayout(_layout);
    _layout->setMargin(0);
    _layout->setSpacing(0);
    _layout->addWidget(_caption);
    _layout->addWidget(_data);
    parent->_dirty=1;
//    QPlastiqueStyle* ps=new QPlastiqueStyle;
//    _caption->setStyle(ps);
//    _data->setStyle(ps);
}

PAbstractDataItem::PAbstractDataItem(PBox* parent, PAbstractDataItem* other) : QWidget(parent), _layout(new QHBoxLayout()),
    _caption(new QLabel(other->caption())), _captionStyle(other->_captionStyle), _defaultDataStyle(other->_defaultDataStyle),
    _lastCapStyle(0), _data(new QLabel(tr("Loading"))), _source(other->_source), _serverDirty(-1)
{
    connect(PStyleNotifier::me,SIGNAL(change()),this,SLOT(pstyleLogic()));
    setLayout(_layout);
    _layout->setMargin(0);
    _layout->setSpacing(0);
    _layout->addWidget(_caption);
    _layout->addWidget(_data);
    parent->_dirty=1;
//    QPlastiqueStyle* ps=new QPlastiqueStyle;
//    _caption->setStyle(ps);
//    _data->setStyle(ps);
}

int PAbstractDataItem::delaysThisCycle() {
  return _delaysThisCycle;
}

void PAbstractDataItem::newCycle() {
  _delaysThisCycle = 0;
}

void PAbstractDataItem::incrementDelays() {
  _delaysThisCycle++;
}

QString PAbstractDataItem::caption() const
{
    return _caption->text();
}

QString PAbstractDataItem::data() const
{
    return _data->text();
}

QString PAbstractDataItem::source() const
{
    return _source;
}

void PAbstractDataItem::mousePressEvent(QMouseEvent *event)
{
    if (PMainWindow::me->mouseInactive()) return;

    emit activated();

    if(event) {
        if (event->button() == Qt::LeftButton) {
            _dragStartPos = event->pos();
        }
    }
}

void PAbstractDataItem::mouseDoubleClickEvent(QMouseEvent*)
{
  QTemporaryFile tmpfile;
  if (tmpfile.open()) { //QIODevice::WriteOnly | QIODevice::Text)) {
    tmpfile.write(QString(
                    "import pykst as kst\n"
                    "client = kst.Client(\""+QString(getenv("USER"))+"-owl\")\n"
                    "x=client.new_data_vector(\""+QString(PMainWindow::me->_dirfileFilename)+"\",field = \"INDEX\", start=-1, num_frames=1000)\n"
                    "y=client.new_data_vector(\""+QString(PMainWindow::me->_dirfileFilename)+"\",field = \""+QString(source())+"\", start=-1, num_frames=1000)\n"
                    "c=client.new_curve(x,y)\n"
                    "p=client.new_plot()\n"
                    "p.add(c)\n"
                    ).toLatin1());
    QString filename = tmpfile.fileName();
    tmpfile.close();
    QProcess::execute ("python2.7 " + filename);
  } else {
    printf("could not write to file!");
  }

  /* // This died.  Don't know why.
  PyRun_SimpleString(QString(
                     "import pykst as kst\n"
                     "client = kst.Client(\""+QString(getenv("USER"))+"-owl\")\n"
                     "x=client.new_data_vector(\""+QString(PMainWindow::me->_dirfileFilename)+"\",field = \"INDEX\", start=-1, num_frames=1000)\n"
                     "y=client.new_data_vector(\""+QString(PMainWindow::me->_dirfileFilename)+"\",field = \""+QString(source())+"\", start=-1, num_frames=1000)\n"
                     "c=client.new_curve(x,y)\n"
                     "p=client.new_plot()\n"
                     "p.add(c)\n"
                     ).toLatin1());
  */

}

void PAbstractDataItem::mouseMoveEvent(QMouseEvent *event)
{
    if (PMainWindow::me->mouseInactive()) return;

    if (!(event->buttons() & Qt::LeftButton))
        return;
    if ((event->pos() - _dragStartPos).manhattanLength() < QApplication::startDragDistance())
        return;

    QDrag *drag = new QDrag(this);
    QMimeData *mimeData = new QMimeData;

    mimeData->setData("text/plain", (PMainWindow::me->_dirfileFilename+"#"+_source).toLatin1());
    mimeData->setData("application/x-owlid", QByteArray::number(id()));
    drag->setMimeData(mimeData);

    drag->exec(Qt::CopyAction | Qt::MoveAction);
}

void PAbstractDataItem::setCaption(QString x, bool force) {
    if(!isCurrentObject()&&!force) {
        return;
    }

    if(_caption->text()==x) {
        return;
    }
    _caption->setText(x);
    emit textChanged(x);
    dynamic_cast<PBox*>(parent())->_dirty=1;
}

void PAbstractDataItem::pstyleLogic() {
    if(_captionStyle!=_lastCapStyle|| //the reason this works is because PStyleChooser works via reference to ptr
            _captionStyle->_dirty) {
        applyStyle(_caption,_captionStyle);
        _lastCapStyle=_captionStyle;
    }
}

void PAbstractDataItem::setSource(QString x,bool force) {
    if(!isCurrentObject()&&!force) return;
    if(x==_source) {
        return;
    }
    _source=x;
    _sourceBad = false;
    _neverGood = true;
    emit sourceChanged(x);
}

void PAbstractDataItem::activate()
{
    emit activated();
}

double PAbstractDataItem::gdReadRawData(GetData::Dirfile* dirFile,int lastNFrames, bool &ok)
{
    double indata;

    ok = false;

    // Read in from disk
    int i=0;
    if (_sourceBad) { // we have determined that this field does not exist in the dirfile, so quit trying.
      return 0.0;
    }
    while (dirFile->GetData(_source.toStdString().c_str(),
                         lastNFrames-1, 0, 0, 1, // 1 sample from frame nf-1
                         GetData::Float64, (void*)(&indata))==0) {

        if (dirFile->Error()== GD_E_BAD_CODE) {
            _sourceBad = true;
            _data->setText("bad src");
            return 0.0;
        }
        if (delaysThisCycle()<=5) {
          if (++i>=5) { // keep trying for 10 x 10000 uS = 0.05s
              if (_neverGood) {
                  if(_data->text()!="bad src") {
                      _data->setText("bad src");
                      _serverDirty=-1;
                      //_sourceBad = true;
                      //qDebug() << "field" << _source << "giving up after 50 tries";
                  } else {
                      --_serverDirty;
                  }
              }
              //qDebug() << "field" << _source << "giving up after 5 tries";
              return 0.0;
          }
          //qDebug() << "field" << _source << "couldn't be read. Sleeping before trying again." << i;
          incrementDelays();
          usleep(10000);
        } else {
          return 0.0;
        }
    }
    _neverGood = false;

    ok = true;
    return (indata);
}
