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

#include "PObject.h"

#include <QColor>
#include <QStringList>
#include <QFont>
#include <QPalette>
#include <QTimer>

#ifndef PFORMAT_H
#define PFORMAT_H

class PStyleNotifier : public QObject
{
    Q_OBJECT
public:
    bool on;
    static PStyleNotifier* me;
    PStyleNotifier() { on=1;}
public slots:
    void disable() { on=0; }
    void enable() { on=1; }
    void notifyChange();
signals:
    void change();
};

class PStyle : public QObject, public PObject
{
    Q_OBJECT
protected:
    QString _name;
    bool _bold, _italic;
    QColor _bg,_fg;
    bool _linked;   /**Set this to true if this PStyle is shared between objects*/

public:
    bool _dirty;
    friend QDataStream& operator<<(QDataStream& a,PStyle& b);
    friend QDataStream& operator>>(QDataStream& a,PStyle& b);
    friend QVariant save(PStyle&);
    friend void load(QVariant v,PStyle&);

    static PStyle* noStyle;
    static QList<PStyle*> _u;
    PStyle(QString name,bool bold=0,bool italic=0,QColor bg="white",QColor fg="black",bool linked=0) :
        _name(name), _bold(bold), _italic(italic),_bg(bg),_fg(fg),_linked(linked), _dirty(1)
    { if(linked) _u.push_back(this); }
    QString name() const { return _name; }
    bool isBold() const { return _bold; }
    bool isItalic() const { return _italic; }
    QColor bgColour() const { return _bg; }
    QColor fgColour() const { return _fg; }
    bool isLinked() const { return _linked; }

public slots:
    void setName(const QString& name) { _name=name; _dirty=1; }
    void setBold(const bool& t) { _bold=t; _dirty=1; }
    void setItalic(const bool& t) { _italic=t; _dirty=1; }
    void setBg(const QColor& c) { _bg=c; _dirty=1; }
    void setFg(const QColor& c) { _fg=c; _dirty=1; }
};

template<class T> void applyStyle(T* obj,PStyle* s)
{
    QFont f=obj->font();
    //f.setFamily("Droid Sans");
    f.setBold(s->isBold());
    f.setItalic(s->isItalic());
    if(obj->font()!=f) obj->setFont(f);
    QPalette p=obj->palette();
    p.setColor(obj->foregroundRole(),s->fgColour());
    p.setColor(obj->backgroundRole(),s->bgColour());
    if(obj->palette()!=p)
    {
        obj->setPalette(p);
        obj->setAutoFillBackground(s->bgColour()!="white");
    }
}

#endif // PFORMAT_H
