/****************************************************************************
** Meta object code from reading C++ file 'SetupView.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.7)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "SetupView.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SetupView.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.7. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_SetupView[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      10,   44,   59,   59, 0x05,

 // slots: signature, parameters, type, tag, flags
      60,   87,   59,   59, 0x08,
      92,   87,   59,   59, 0x08,
     118,   59,   59,   59, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_SetupView[] = {
    "SetupView\0doneSetup(GetData::Dirfile*,json)\0"
    "dirfile,config\0\0updateDirfilePath(QString)\0"
    "path\0updateConfigPath(QString)\0"
    "checkFiles()\0"
};

void SetupView::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        SetupView *_t = static_cast<SetupView *>(_o);
        switch (_id) {
        case 0: _t->doneSetup((*reinterpret_cast< GetData::Dirfile*(*)>(_a[1])),(*reinterpret_cast< json(*)>(_a[2]))); break;
        case 1: _t->updateDirfilePath((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 2: _t->updateConfigPath((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 3: _t->checkFiles(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData SetupView::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject SetupView::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_SetupView,
      qt_meta_data_SetupView, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &SetupView::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *SetupView::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *SetupView::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_SetupView))
        return static_cast<void*>(const_cast< SetupView*>(this));
    return QWidget::qt_metacast(_clname);
}

int SetupView::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void SetupView::doneSetup(GetData::Dirfile * _t1, json _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
