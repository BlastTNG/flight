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
#include <QCompleter>

class AbstractNarsilEntry
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
    virtual QString Text() const=0;
protected:
    int command, param;
    char type;
};

class NarsilStringEntry : public QLineEdit, public AbstractNarsilEntry
{
    Q_OBJECT
public:
    NarsilStringEntry(QWidget* parent, QString objName) : QLineEdit(parent)
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
#include <iostream>
class NLineEdit : public QLineEdit
{
    Q_OBJECT
protected:
    void mouseDoubleClickEvent(QMouseEvent *) {
        selectAll();
    }
};

class NarsilDoubleEntry : public QDoubleSpinBox, public AbstractNarsilEntry
{
    Q_OBJECT
public:
    NarsilDoubleEntry(QWidget* parent, QString objName) : QDoubleSpinBox(parent)
    {
//        delete lineEdit();
        setLineEdit(new NLineEdit);
        setObjectName(objName);
        setMinimum(-__DBL_MAX__);
        setMaximum(__DBL_MAX__);
    }

    void SetType(char t)
    {
        Q_ASSERT(t!='s');
        setDecimals((t=='i'?0:4));
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
};

#include <QKeyEvent>

class NarsilOmniBox : public QLineEdit
{
    Q_OBJECT
    int oldXSize;
public:
    friend class MainForm;
    NarsilOmniBox(QWidget*p) : QLineEdit(p), oldXSize(0) {}
};

#endif // WIDGETS_H
