/****************************************************************************
** DoubleEntry meta object code from reading C++ file 'doubleentry.h'
**
** Created: Sat Apr 12 19:15:59 2003
**      by: The Qt MOC ($Id: moc_doubleentry.cpp,v 1.1.1.1 2003-06-16 21:17:59 dwiebe Exp $)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#undef QT_NO_COMPAT
#include "doubleentry.h"
#include <qmetaobject.h>
#include <qapplication.h>

#include <private/qucomextra_p.h>
#if !defined(Q_MOC_OUTPUT_REVISION) || (Q_MOC_OUTPUT_REVISION != 26)
#error "This file was generated using the moc from 3.1.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

const char *DoubleEntry::className() const
{
    return "DoubleEntry";
}

QMetaObject *DoubleEntry::metaObj = 0;
static QMetaObjectCleanUp cleanUp_DoubleEntry( "DoubleEntry", &DoubleEntry::staticMetaObject );

#ifndef QT_NO_TRANSLATION
QString DoubleEntry::tr( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "DoubleEntry", s, c, QApplication::DefaultCodec );
    else
	return QString::fromLatin1( s );
}
#ifndef QT_NO_TRANSLATION_UTF8
QString DoubleEntry::trUtf8( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "DoubleEntry", s, c, QApplication::UnicodeUTF8 );
    else
	return QString::fromUtf8( s );
}
#endif // QT_NO_TRANSLATION_UTF8

#endif // QT_NO_TRANSLATION

QMetaObject* DoubleEntry::staticMetaObject()
{
    if ( metaObj )
	return metaObj;
    QMetaObject* parentObject = QLineEdit::staticMetaObject();
    metaObj = QMetaObject::new_metaobject(
	"DoubleEntry", parentObject,
	0, 0,
	0, 0,
#ifndef QT_NO_PROPERTIES
	0, 0,
	0, 0,
#endif // QT_NO_PROPERTIES
	0, 0 );
    cleanUp_DoubleEntry.setMetaObject( metaObj );
    return metaObj;
}

void* DoubleEntry::qt_cast( const char* clname )
{
    if ( !qstrcmp( clname, "DoubleEntry" ) )
	return this;
    return QLineEdit::qt_cast( clname );
}

bool DoubleEntry::qt_invoke( int _id, QUObject* _o )
{
    return QLineEdit::qt_invoke(_id,_o);
}

bool DoubleEntry::qt_emit( int _id, QUObject* _o )
{
    return QLineEdit::qt_emit(_id,_o);
}
#ifndef QT_NO_PROPERTIES

bool DoubleEntry::qt_property( int id, int f, QVariant* v)
{
    return QLineEdit::qt_property( id, f, v);
}

bool DoubleEntry::qt_static_property( QObject* , int , int , QVariant* ){ return FALSE; }
#endif // QT_NO_PROPERTIES
