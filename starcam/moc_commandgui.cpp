/****************************************************************************
** CommandGUI meta object code from reading C++ file 'commandgui.h'
**
** Created: Fri May 4 14:09:30 2007
**      by: The Qt MOC ($Id: moc_commandgui.cpp,v 1.1.1.1 2007-07-04 21:33:40 steve Exp $)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#undef QT_NO_COMPAT
#include "commandgui.h"
#include <qmetaobject.h>
#include <qapplication.h>

#include <private/qucomextra_p.h>
#if !defined(Q_MOC_OUTPUT_REVISION) || (Q_MOC_OUTPUT_REVISION != 26)
#error "This file was generated using the moc from 3.3.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

const char *CommandGUI::className() const
{
    return "CommandGUI";
}

QMetaObject *CommandGUI::metaObj = 0;
static QMetaObjectCleanUp cleanUp_CommandGUI( "CommandGUI", &CommandGUI::staticMetaObject );

#ifndef QT_NO_TRANSLATION
QString CommandGUI::tr( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "CommandGUI", s, c, QApplication::DefaultCodec );
    else
	return QString::fromLatin1( s );
}
#ifndef QT_NO_TRANSLATION_UTF8
QString CommandGUI::trUtf8( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "CommandGUI", s, c, QApplication::UnicodeUTF8 );
    else
	return QString::fromUtf8( s );
}
#endif // QT_NO_TRANSLATION_UTF8

#endif // QT_NO_TRANSLATION

QMetaObject* CommandGUI::staticMetaObject()
{
    if ( metaObj )
	return metaObj;
    QMetaObject* parentObject = QWidget::staticMetaObject();
    static const QUParameter param_slot_0[] = {
	{ 0, &static_QUType_int, 0, QUParameter::In }
    };
    static const QUMethod slot_0 = {"deviceSelected", 1, param_slot_0 };
    static const QUParameter param_slot_1[] = {
	{ 0, &static_QUType_int, 0, QUParameter::In }
    };
    static const QUMethod slot_1 = {"commandSelected", 1, param_slot_1 };
    static const QUParameter param_slot_2[] = {
	{ 0, &static_QUType_QString, 0, QUParameter::In }
    };
    static const QUMethod slot_2 = {"valueChanged", 1, param_slot_2 };
    static const QUMethod slot_3 = {"sendCommand", 0, 0 };
    static const QMetaData slot_tbl[] = {
	{ "deviceSelected(int)", &slot_0, QMetaData::Public },
	{ "commandSelected(int)", &slot_1, QMetaData::Public },
	{ "valueChanged(const QString&)", &slot_2, QMetaData::Public },
	{ "sendCommand()", &slot_3, QMetaData::Public }
    };
    metaObj = QMetaObject::new_metaobject(
	"CommandGUI", parentObject,
	slot_tbl, 4,
	0, 0,
#ifndef QT_NO_PROPERTIES
	0, 0,
	0, 0,
#endif // QT_NO_PROPERTIES
	0, 0 );
    cleanUp_CommandGUI.setMetaObject( metaObj );
    return metaObj;
}

void* CommandGUI::qt_cast( const char* clname )
{
    if ( !qstrcmp( clname, "CommandGUI" ) )
	return this;
    return QWidget::qt_cast( clname );
}

bool CommandGUI::qt_invoke( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->slotOffset() ) {
    case 0: deviceSelected((int)static_QUType_int.get(_o+1)); break;
    case 1: commandSelected((int)static_QUType_int.get(_o+1)); break;
    case 2: valueChanged((const QString&)static_QUType_QString.get(_o+1)); break;
    case 3: sendCommand(); break;
    default:
	return QWidget::qt_invoke( _id, _o );
    }
    return TRUE;
}

bool CommandGUI::qt_emit( int _id, QUObject* _o )
{
    return QWidget::qt_emit(_id,_o);
}
#ifndef QT_NO_PROPERTIES

bool CommandGUI::qt_property( int id, int f, QVariant* v)
{
    return QWidget::qt_property( id, f, v);
}

bool CommandGUI::qt_static_property( QObject* , int , int , QVariant* ){ return FALSE; }
#endif // QT_NO_PROPERTIES
