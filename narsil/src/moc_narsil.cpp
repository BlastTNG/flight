/****************************************************************************
** MainForm meta object code from reading C++ file 'narsil.h'
**
** Created: Tue Jun 17 19:47:19 2003
**      by: The Qt MOC ($Id: moc_narsil.cpp,v 1.2 2003-06-17 23:49:10 dwiebe Exp $)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#undef QT_NO_COMPAT
#include "narsil.h"
#include <qmetaobject.h>
#include <qapplication.h>

#include <private/qucomextra_p.h>
#if !defined(Q_MOC_OUTPUT_REVISION) || (Q_MOC_OUTPUT_REVISION != 26)
#error "This file was generated using the moc from 3.1.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

const char *MainForm::className() const
{
    return "MainForm";
}

QMetaObject *MainForm::metaObj = 0;
static QMetaObjectCleanUp cleanUp_MainForm( "MainForm", &MainForm::staticMetaObject );

#ifndef QT_NO_TRANSLATION
QString MainForm::tr( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "MainForm", s, c, QApplication::DefaultCodec );
    else
	return QString::fromLatin1( s );
}
#ifndef QT_NO_TRANSLATION_UTF8
QString MainForm::trUtf8( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "MainForm", s, c, QApplication::UnicodeUTF8 );
    else
	return QString::fromUtf8( s );
}
#endif // QT_NO_TRANSLATION_UTF8

#endif // QT_NO_TRANSLATION

QMetaObject* MainForm::staticMetaObject()
{
    if ( metaObj )
	return metaObj;
    QMetaObject* parentObject = QDialog::staticMetaObject();
    static const QUMethod slot_0 = {"ChangeCommandList", 0, 0 };
    static const QUMethod slot_1 = {"ChooseCommand", 0, 0 };
    static const QUMethod slot_2 = {"Quit", 0, 0 };
    static const QUMethod slot_3 = {"SendCommand", 0, 0 };
    static const QUMethod slot_4 = {"ChangeImage", 0, 0 };
    static const QUMethod slot_5 = {"ChangeCurFile", 0, 0 };
    static const QUMethod slot_6 = {"ShowSettings", 0, 0 };
    static const QMetaData slot_tbl[] = {
	{ "ChangeCommandList()", &slot_0, QMetaData::Public },
	{ "ChooseCommand()", &slot_1, QMetaData::Public },
	{ "Quit()", &slot_2, QMetaData::Public },
	{ "SendCommand()", &slot_3, QMetaData::Public },
	{ "ChangeImage()", &slot_4, QMetaData::Public },
	{ "ChangeCurFile()", &slot_5, QMetaData::Public },
	{ "ShowSettings()", &slot_6, QMetaData::Public }
    };
    metaObj = QMetaObject::new_metaobject(
	"MainForm", parentObject,
	slot_tbl, 7,
	0, 0,
#ifndef QT_NO_PROPERTIES
	0, 0,
	0, 0,
#endif // QT_NO_PROPERTIES
	0, 0 );
    cleanUp_MainForm.setMetaObject( metaObj );
    return metaObj;
}

void* MainForm::qt_cast( const char* clname )
{
    if ( !qstrcmp( clname, "MainForm" ) )
	return this;
    return QDialog::qt_cast( clname );
}

bool MainForm::qt_invoke( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->slotOffset() ) {
    case 0: ChangeCommandList(); break;
    case 1: ChooseCommand(); break;
    case 2: Quit(); break;
    case 3: SendCommand(); break;
    case 4: ChangeImage(); break;
    case 5: ChangeCurFile(); break;
    case 6: ShowSettings(); break;
    default:
	return QDialog::qt_invoke( _id, _o );
    }
    return TRUE;
}

bool MainForm::qt_emit( int _id, QUObject* _o )
{
    return QDialog::qt_emit(_id,_o);
}
#ifndef QT_NO_PROPERTIES

bool MainForm::qt_property( int id, int f, QVariant* v)
{
    return QDialog::qt_property( id, f, v);
}

bool MainForm::qt_static_property( QObject* , int , int , QVariant* ){ return FALSE; }
#endif // QT_NO_PROPERTIES
