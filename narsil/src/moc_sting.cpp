/****************************************************************************
** FloatSpinBox meta object code from reading C++ file 'sting.h'
**
** Created: Fri Jun 7 17:17:36 2002
**      by: The Qt MOC ($Id: moc_sting.cpp,v 1.1.1.1 2003-06-16 21:17:59 dwiebe Exp $)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#if !defined(Q_MOC_OUTPUT_REVISION)
#define Q_MOC_OUTPUT_REVISION 9
#elif Q_MOC_OUTPUT_REVISION != 9
#error "Moc format conflict - please regenerate all moc files"
#endif

#include "sting.h"
#include <qmetaobject.h>
#include <qapplication.h>



const char *FloatSpinBox::className() const
{
    return "FloatSpinBox";
}

QMetaObject *FloatSpinBox::metaObj = 0;

void FloatSpinBox::initMetaObject()
{
    if ( metaObj )
	return;
    if ( qstrcmp(QSpinBox::className(), "QSpinBox") != 0 )
	badSuperclassWarning("FloatSpinBox","QSpinBox");
    (void) staticMetaObject();
}

#ifndef QT_NO_TRANSLATION

QString FloatSpinBox::tr(const char* s)
{
    return qApp->translate( "FloatSpinBox", s, 0 );
}

QString FloatSpinBox::tr(const char* s, const char * c)
{
    return qApp->translate( "FloatSpinBox", s, c );
}

#endif // QT_NO_TRANSLATION

QMetaObject* FloatSpinBox::staticMetaObject()
{
    if ( metaObj )
	return metaObj;
    (void) QSpinBox::staticMetaObject();
#ifndef QT_NO_PROPERTIES
#endif // QT_NO_PROPERTIES
    QMetaData::Access *slot_tbl_access = 0;
    metaObj = QMetaObject::new_metaobject(
	"FloatSpinBox", "QSpinBox",
	0, 0,
	0, 0,
#ifndef QT_NO_PROPERTIES
	0, 0,
	0, 0,
#endif // QT_NO_PROPERTIES
	0, 0 );
    metaObj->set_slot_access( slot_tbl_access );
#ifndef QT_NO_PROPERTIES
#endif // QT_NO_PROPERTIES
    return metaObj;
}


const char *MainForm::className() const
{
    return "MainForm";
}

QMetaObject *MainForm::metaObj = 0;

void MainForm::initMetaObject()
{
    if ( metaObj )
	return;
    if ( qstrcmp(QDialog::className(), "QDialog") != 0 )
	badSuperclassWarning("MainForm","QDialog");
    (void) staticMetaObject();
}

#ifndef QT_NO_TRANSLATION

QString MainForm::tr(const char* s)
{
    return qApp->translate( "MainForm", s, 0 );
}

QString MainForm::tr(const char* s, const char * c)
{
    return qApp->translate( "MainForm", s, c );
}

#endif // QT_NO_TRANSLATION

QMetaObject* MainForm::staticMetaObject()
{
    if ( metaObj )
	return metaObj;
    (void) QDialog::staticMetaObject();
#ifndef QT_NO_PROPERTIES
#endif // QT_NO_PROPERTIES
    typedef void (MainForm::*m1_t0)();
    typedef void (QObject::*om1_t0)();
    typedef void (MainForm::*m1_t1)();
    typedef void (QObject::*om1_t1)();
    typedef void (MainForm::*m1_t2)();
    typedef void (QObject::*om1_t2)();
    typedef void (MainForm::*m1_t3)();
    typedef void (QObject::*om1_t3)();
    m1_t0 v1_0 = &MainForm::ChangeCommandList;
    om1_t0 ov1_0 = (om1_t0)v1_0;
    m1_t1 v1_1 = &MainForm::ChooseCommand;
    om1_t1 ov1_1 = (om1_t1)v1_1;
    m1_t2 v1_2 = &MainForm::Quit;
    om1_t2 ov1_2 = (om1_t2)v1_2;
    m1_t3 v1_3 = &MainForm::SendCommand;
    om1_t3 ov1_3 = (om1_t3)v1_3;
    QMetaData *slot_tbl = QMetaObject::new_metadata(4);
    QMetaData::Access *slot_tbl_access = QMetaObject::new_metaaccess(4);
    slot_tbl[0].name = "ChangeCommandList()";
    slot_tbl[0].ptr = (QMember)ov1_0;
    slot_tbl_access[0] = QMetaData::Public;
    slot_tbl[1].name = "ChooseCommand()";
    slot_tbl[1].ptr = (QMember)ov1_1;
    slot_tbl_access[1] = QMetaData::Public;
    slot_tbl[2].name = "Quit()";
    slot_tbl[2].ptr = (QMember)ov1_2;
    slot_tbl_access[2] = QMetaData::Public;
    slot_tbl[3].name = "SendCommand()";
    slot_tbl[3].ptr = (QMember)ov1_3;
    slot_tbl_access[3] = QMetaData::Public;
    metaObj = QMetaObject::new_metaobject(
	"MainForm", "QDialog",
	slot_tbl, 4,
	0, 0,
#ifndef QT_NO_PROPERTIES
	0, 0,
	0, 0,
#endif // QT_NO_PROPERTIES
	0, 0 );
    metaObj->set_slot_access( slot_tbl_access );
#ifndef QT_NO_PROPERTIES
#endif // QT_NO_PROPERTIES
    return metaObj;
}
