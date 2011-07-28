/* narsil: GUI commanding front-end
 *
 * This software is copyright (C) 2002-2006 University of Toronto
 * Parts of this software are copyright 2010 Matthew Truch
 *
 * This file is part of narsil.
 *
 * narsil is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * narsil is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with narsil; if not, write to the Free Software Foundation, Inc.,
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

#include "narsil.h"

#include <QLineEdit>
#include <QTextEdit>

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
    virtual QString Text()=0;
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
    QString Text()
    {
        return text();
    }
};
#include <iostream>
class NarsilDoubleEntry : public QDoubleSpinBox, public AbstractNarsilEntry
{
    Q_OBJECT
public:
    NarsilDoubleEntry(QWidget* parent, QString objName) : QDoubleSpinBox(parent)
    {
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

    QString Text()
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
        bool ok;
        int i;
        for(i=0;i<input.size();i++) {
            if(i+2!=input.size()&&input[i+1]!='.'&&input[i]=='0') {
                input.remove(i--,1);
                continue;
            } else if(input[i]=='.') {
                break;
            }
        }
        for(++i;i<input.size();i++) {
            if(input[i]=='.') {
                input.remove(i--,1);
                continue;
            }
        }
        if(input.toDouble(&ok),ok) {
            return QValidator::Acceptable;
        }
    }
};

#include <QKeyEvent>

class NarsilOmniBox : public QLineEdit
{
    Q_OBJECT
public:
    friend class MainForm;
    NarsilOmniBox(QWidget*p) : QLineEdit(p) {}
protected:
    void keyPressEvent(QKeyEvent *ev) { // reimplementing because (1) we want to ensure that we have focus (2) the way Qt treats selections isn't all that
                                        // great for our purposes.
        setFocus();
        if(ev->key()==Qt::Key_Left) {
            setCursorPosition(qMin((selectionStart()==-1)?1000000:selectionStart(),cursorPosition())-1);
        } else if(ev->key()==Qt::Key_Right) {
            setCursorPosition(qMin((selectionStart()==-1)?1000000:selectionStart(),cursorPosition())+1);
        } else {
            QLineEdit::keyPressEvent(ev);
        }
    }
};

#endif // WIDGETS_H
