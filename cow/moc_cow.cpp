/****************************************************************************
** Meta object code from reading C++ file 'cow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "src/cow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'cow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_MainForm_t {
    QByteArrayData data[21];
    char stringdata0[232];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainForm_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainForm_t qt_meta_stringdata_MainForm = {
    {
QT_MOC_LITERAL(0, 0, 8), // "MainForm"
QT_MOC_LITERAL(1, 9, 9), // "OmniParse"
QT_MOC_LITERAL(2, 19, 0), // ""
QT_MOC_LITERAL(3, 20, 6), // "filter"
QT_MOC_LITERAL(4, 27, 8), // "OmniSync"
QT_MOC_LITERAL(5, 36, 17), // "ChangeCommandList"
QT_MOC_LITERAL(6, 54, 6), // "really"
QT_MOC_LITERAL(7, 61, 17), // "IndexComboChanged"
QT_MOC_LITERAL(8, 79, 1), // "i"
QT_MOC_LITERAL(9, 81, 13), // "ChooseCommand"
QT_MOC_LITERAL(10, 95, 19), // "index_combo_changed"
QT_MOC_LITERAL(11, 115, 11), // "combo_index"
QT_MOC_LITERAL(12, 127, 4), // "Quit"
QT_MOC_LITERAL(13, 132, 11), // "SendCommand"
QT_MOC_LITERAL(14, 144, 4), // "Tick"
QT_MOC_LITERAL(15, 149, 4), // "Ping"
QT_MOC_LITERAL(16, 154, 10), // "ChangeHost"
QT_MOC_LITERAL(17, 165, 13), // "ServerDropped"
QT_MOC_LITERAL(18, 179, 27), // "nOmniBox_completerActivated"
QT_MOC_LITERAL(19, 207, 4), // "text"
QT_MOC_LITERAL(20, 212, 19) // "nOmniBox_textEdited"

    },
    "MainForm\0OmniParse\0\0filter\0OmniSync\0"
    "ChangeCommandList\0really\0IndexComboChanged\0"
    "i\0ChooseCommand\0index_combo_changed\0"
    "combo_index\0Quit\0SendCommand\0Tick\0"
    "Ping\0ChangeHost\0ServerDropped\0"
    "nOmniBox_completerActivated\0text\0"
    "nOmniBox_textEdited"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainForm[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      17,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   99,    2, 0x0a /* Public */,
       1,    0,  102,    2, 0x2a /* Public | MethodCloned */,
       4,    0,  103,    2, 0x0a /* Public */,
       5,    1,  104,    2, 0x0a /* Public */,
       5,    0,  107,    2, 0x2a /* Public | MethodCloned */,
       7,    1,  108,    2, 0x0a /* Public */,
       9,    2,  111,    2, 0x0a /* Public */,
       9,    1,  116,    2, 0x2a /* Public | MethodCloned */,
       9,    0,  119,    2, 0x2a /* Public | MethodCloned */,
      12,    0,  120,    2, 0x0a /* Public */,
      13,    0,  121,    2, 0x0a /* Public */,
      14,    0,  122,    2, 0x0a /* Public */,
      15,    0,  123,    2, 0x0a /* Public */,
      16,    0,  124,    2, 0x0a /* Public */,
      17,    0,  125,    2, 0x0a /* Public */,
      18,    1,  126,    2, 0x0a /* Public */,
      20,    1,  129,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,    6,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    8,
    QMetaType::Void, QMetaType::Bool, QMetaType::Int,   10,   11,
    QMetaType::Void, QMetaType::Bool,   10,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   19,
    QMetaType::Void, QMetaType::QString,   19,

       0        // eod
};

void MainForm::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MainForm *_t = static_cast<MainForm *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->OmniParse((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 1: _t->OmniParse(); break;
        case 2: _t->OmniSync(); break;
        case 3: _t->ChangeCommandList((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 4: _t->ChangeCommandList(); break;
        case 5: _t->IndexComboChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: _t->ChooseCommand((*reinterpret_cast< bool(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 7: _t->ChooseCommand((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 8: _t->ChooseCommand(); break;
        case 9: _t->Quit(); break;
        case 10: _t->SendCommand(); break;
        case 11: _t->Tick(); break;
        case 12: _t->Ping(); break;
        case 13: _t->ChangeHost(); break;
        case 14: _t->ServerDropped(); break;
        case 15: _t->nOmniBox_completerActivated((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 16: _t->nOmniBox_textEdited((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject MainForm::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainForm.data,
      qt_meta_data_MainForm,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *MainForm::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainForm::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_MainForm.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int MainForm::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 17)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 17;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 17)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 17;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
