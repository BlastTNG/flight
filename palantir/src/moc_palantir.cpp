/****************************************************************************
** MainForm meta object code from reading C++ file 'palantir.h'
**
** Created: Mon May 19 18:24:54 2003
**      by: The Qt MOC ($Id: moc_palantir.cpp,v 1.1.1.1 2003-06-16 21:36:47 dwiebe Exp $)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#undef QT_NO_COMPAT
#include "palantir.h"
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
    static const QUMethod slot_0 = {"UpdateData", 0, 0 };
    static const QUMethod slot_1 = {"ChangeCurFile", 0, 0 };
    static const QUMethod slot_2 = {"ShowAlarms", 0, 0 };
    static const QUMethod slot_3 = {"ChangeChooseSound", 0, 0 };
    static const QUMethod slot_4 = {"ReactivateAllAlarms", 0, 0 };
    static const QUMethod slot_5 = {"ReactivateAlarm", 0, 0 };
    static const QMetaData slot_tbl[] = {
	{ "UpdateData()", &slot_0, QMetaData::Public },
	{ "ChangeCurFile()", &slot_1, QMetaData::Public },
	{ "ShowAlarms()", &slot_2, QMetaData::Public },
	{ "ChangeChooseSound()", &slot_3, QMetaData::Public },
	{ "ReactivateAllAlarms()", &slot_4, QMetaData::Public },
	{ "ReactivateAlarm()", &slot_5, QMetaData::Public }
    };
    metaObj = QMetaObject::new_metaobject(
	"MainForm", parentObject,
	slot_tbl, 6,
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
    case 0: UpdateData(); break;
    case 1: ChangeCurFile(); break;
    case 2: ShowAlarms(); break;
    case 3: ChangeChooseSound(); break;
    case 4: ReactivateAllAlarms(); break;
    case 5: ReactivateAlarm(); break;
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
