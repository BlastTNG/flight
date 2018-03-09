/* cow: GUI commanding front-end
 *
 * This software is copyright (C) 2002-2006 University of Toronto
 * Parts of this software are copyright 2010 Matthew Truch
 *
 * This file is part of cow.
 *
 * cow is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * cow is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with cow; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

// ***************************************************
// *  Programmed by Adam Hincks                      *
// *  Later poked at a bit by D.V.Wiebe              *
// *  further desecrated by cbn                      *
// *  Commanding hacked to hell by M.D.P.Truch       *
// *  "Ported" to qt4 by drmrshdw                    *
// ***************************************************

#ifndef WIDGETS_H
#define WIDGETS_H

#include "cow.h"

#include <QLineEdit>
#include <QTextEdit>
#include <QDebug>
#include <QComboBox>
#include <QCompleter>

class AbstractCowEntry
{
public:
    virtual void RecordDefaults()    //fill default element
    {
      defaults->Set(command, param, Text());
    }

    void SetDefaultValue(int i, int j) {
      if (type == 'i' || type == 'l')
        SetValue(defaults->asInt(i, j));
      else if (type == 's')
        SetStringValue(defaults->asString(i, j));
      else
        SetValue(defaults->asDouble(i, j));
    }

    virtual void SetParentField(int com, int par)
    {
      command = com;
      param = par;
    }

    virtual void SetType(char t)=0;
    virtual void SetValue(double d)=0;
    virtual void SetStringValue(QString s)=0;
    virtual void SetIndex(int i) {Q_UNUSED(i)}
    virtual QString Text() const=0;
protected:
    int command, param;
    char type;
};

class CowStringEntry : public QLineEdit, public AbstractCowEntry
{
    Q_OBJECT
public:
    CowStringEntry(QWidget* parent, QString objName) : QLineEdit(parent)
    {
        setObjectName(objName);
    }
    virtual void SetType(char t)
    {
        Q_UNUSED(t);
        Q_ASSERT(t=='s');
    }
    virtual void SetValue(double d)
    {
        Q_UNUSED(d);
        qFatal("Depricated call to NarcilStringEntry::SetValue(...).");
    }
    void SetStringValue(QString s)
    {
        setText(s);
    }
    QString Text() const
    {
        return text();
    }
};

class CowComboEntry : public QComboBox, public AbstractCowEntry
{
    Q_OBJECT
  public:
    CowComboEntry(QWidget* parent, QString objName) : QComboBox(parent)
    {
      minVal = 0;
      setObjectName(objName);
      connect(this, SIGNAL(currentIndexChanged(int)), this, SLOT(slotValueChanged()));
    }
    virtual void SetType(char t)
    {
      Q_ASSERT(t!='s');
      type = t;
    }
    virtual void SetValue(double d)
    {
      SetValue(int(d));
    }

    virtual void SetValue(int i)
    {

      i -= minVal;

      i = qMin(i, count()-1);
      i = qMax(i,0);

      setCurrentIndex(i);
    }

    void SetStringValue(QString s)
    {
      bool ok;
      int i = s.toInt(&ok);
      if (!ok) {
        i = findText(s);
      }
      SetValue(i);
    }

    virtual void SetIndex(int i)
    {
      setCurrentIndex(i);
    }

    QString Text() const
    {

      return QString::number(currentIndex() + minVal);
    }

    int minVal;

  public slots:
    void slotValueChanged()
    {   //to override parent's valueChanged signal to require focus
      if (hasFocus()) {
        emit valueEdited();
      }
    }

  signals:
    void valueEdited();

};

#include <iostream>
class NLineEdit : public QLineEdit
{
    Q_OBJECT
protected:
    void mouseDoubleClickEvent(QMouseEvent *) {
        selectAll();
    }
};

class CowDoubleEntry : public QDoubleSpinBox, public AbstractCowEntry
{
    Q_OBJECT
public:
    CowDoubleEntry(QWidget* parent, QString objName) : QDoubleSpinBox(parent)
    {
//        delete lineEdit();
        setLineEdit(new NLineEdit);
        setObjectName(objName);
        setMinimum(-__DBL_MAX__);
        setMaximum(__DBL_MAX__);
        connect(this, SIGNAL(valueChanged(double)), this, SLOT(slotValueChanged()));
    }

    void SetType(char t)
    {
        Q_ASSERT(t!='s');
        setDecimals(((t=='i' || t=='l') ? 0 : 5));
        type = t;
    }

    void SetValue(double d)
    {
        setValue(d);
    }

    QString Text() const
    {
        return text();
    }

    double value() const
    {
        Q_ASSERT(type!='s');
        return QDoubleSpinBox::value();
    }

    void SetStringValue(QString s)
    {
        bool ok=0;
        SetValue(s.toDouble(&ok));
        if(!ok) {
            SetValue(s.toInt(&ok));
        }
        Q_ASSERT(ok);
    }
    QValidator::State validate(QString &input, int &pos) const
    {
        if(input.indexOf('.')!=-1) {
            QString x=input;
            x.remove(0,x.indexOf('.')+1);
            input.chop(qMax(0,x.size()-decimals()));
        }
        return QDoubleSpinBox::validate(input,pos);
    }

public slots:
    void slotValueChanged()
    {   //to override parent's valueChanged signal to require focus
        if (hasFocus()) {
            emit valueEdited();
        }
    }

signals:
    void valueEdited();

};

class CowOmniBox : public QLineEdit
{
    Q_OBJECT
    int oldXSize;

    //hack to prevent QCompleter from overwriting the box text: track real text separately
    QString _realText;

public:
    friend class MainForm;
    CowOmniBox(QWidget*p) : QLineEdit(p), oldXSize(0), _realText() {
        connect(this,SIGNAL(textChanged(QString)),this,SLOT(updateRealText()));
        connect(this,SIGNAL(realTextChanged()),this,SLOT(updateRealText()));
    }

    void setRealText(const QString &text) {
        _realText = text;
        setText(text);
    }
    QString & realText() { return _realText; }

public slots:
    void updateRealText() { //connect to textChanged to prevent Qt from resetting it
        if (text() != realText()) {
            setText(realText());
            emit realTextChanged();
        }
    }

signals:
    void realTextChanged();
};

#endif // WIDGETS_H
