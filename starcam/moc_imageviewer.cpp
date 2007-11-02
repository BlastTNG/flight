/****************************************************************************
** ImageViewer meta object code from reading C++ file 'imageviewer.h'
**
** Created: Fri Nov 2 11:34:34 2007
**      by: The Qt MOC ($Id: moc_imageviewer.cpp,v 1.3 2007-11-02 15:37:00 steve Exp $)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#undef QT_NO_COMPAT
#include "imageviewer.h"
#include <qmetaobject.h>
#include <qapplication.h>

#include <private/qucomextra_p.h>
#if !defined(Q_MOC_OUTPUT_REVISION) || (Q_MOC_OUTPUT_REVISION != 26)
#error "This file was generated using the moc from 3.3.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

const char *ImageViewer::className() const
{
    return "ImageViewer";
}

QMetaObject *ImageViewer::metaObj = 0;
static QMetaObjectCleanUp cleanUp_ImageViewer( "ImageViewer", &ImageViewer::staticMetaObject );

#ifndef QT_NO_TRANSLATION
QString ImageViewer::tr( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "ImageViewer", s, c, QApplication::DefaultCodec );
    else
	return QString::fromLatin1( s );
}
#ifndef QT_NO_TRANSLATION_UTF8
QString ImageViewer::trUtf8( const char *s, const char *c )
{
    if ( qApp )
	return qApp->translate( "ImageViewer", s, c, QApplication::UnicodeUTF8 );
    else
	return QString::fromUtf8( s );
}
#endif // QT_NO_TRANSLATION_UTF8

#endif // QT_NO_TRANSLATION

QMetaObject* ImageViewer::staticMetaObject()
{
    if ( metaObj )
	return metaObj;
    QMetaObject* parentObject = QWidget::staticMetaObject();
    static const QUMethod slot_0 = {"refresh", 0, 0 };
    static const QMetaData slot_tbl[] = {
	{ "refresh()", &slot_0, QMetaData::Public }
    };
    metaObj = QMetaObject::new_metaobject(
	"ImageViewer", parentObject,
	slot_tbl, 1,
	0, 0,
#ifndef QT_NO_PROPERTIES
	0, 0,
	0, 0,
#endif // QT_NO_PROPERTIES
	0, 0 );
    cleanUp_ImageViewer.setMetaObject( metaObj );
    return metaObj;
}

void* ImageViewer::qt_cast( const char* clname )
{
    if ( !qstrcmp( clname, "ImageViewer" ) )
	return this;
    return QWidget::qt_cast( clname );
}

bool ImageViewer::qt_invoke( int _id, QUObject* _o )
{
    switch ( _id - staticMetaObject()->slotOffset() ) {
    case 0: refresh(); break;
    default:
	return QWidget::qt_invoke( _id, _o );
    }
    return TRUE;
}

bool ImageViewer::qt_emit( int _id, QUObject* _o )
{
    return QWidget::qt_emit(_id,_o);
}
#ifndef QT_NO_PROPERTIES

bool ImageViewer::qt_property( int id, int f, QVariant* v)
{
    return QWidget::qt_property( id, f, v);
}

bool ImageViewer::qt_static_property( QObject* , int , int , QVariant* ){ return FALSE; }
#endif // QT_NO_PROPERTIES
