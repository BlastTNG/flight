/* palantir: BLAST status GUI
 *
 * This software is copyright (C) 2002-2004 University of Toronto
 * 
 * This file is part of palantir.
 * 
 * palantir is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * palantir is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with palantir; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

// ***************************************************
// *  Programmed by Adam Hincks et al.               *
// *                                                 *
// *  Comments on classes & functions in .cpp file   *
// ***************************************************

#ifndef PALANTIR_H
#define PALANTIR_H

#include <qapplication.h>
#include <qlabel.h>
#include <qpushbutton.h>
#include <qlayout.h>
#include <qvariant.h>
#include <qtooltip.h>
#include <qthread.h>
#include <qwhatsthis.h>
#include <qmessagebox.h>
#include <qgroupbox.h>
#include <qdom.h>
#include <qfile.h>
#include <qcolor.h>
#include <qvariant.h>
#include <qdialog.h>
#include <qmainwindow.h>
#include <qlist.h>
#include <qpixmap.h>
#include <qcombobox.h>
#include <qframe.h>
#include <qlistbox.h>
#include <qmultilineedit.h>
#include <qstatusbar.h>
#include <unistd.h>

#define NUMBER       0
#define MULTI        1
#define DATETIME     2
#define DERIV        3
#define CURDIR       4

// Update palantir every 750 milliseconds
#define UPDATETIME 500

#ifndef LIB_DIR
#  define LIB_DIR "/usr/local/lib/palantir/"
#endif

#ifndef DEF_LAYOUTFILE
#  define DEF_LAYOUTFILE LIB_DIR "default.pal"
#endif

#ifndef LOGFILE
#  define LOGFILE LIB_DIR "log.txt"
#endif

#define MAXPATHLENGTH 255
#define MAX_MULTI_WORDS 255

#define PALANTIR_0_JPG LIB_DIR "palantir0.jpg"
#define PALANTIR_1_JPG LIB_DIR "palantir1.jpg"
#define PALANTIR_2_JPG LIB_DIR "palantir2.jpg"
#define PALANTIR_3_JPG LIB_DIR "palantir3.jpg"


#define ICON LIB_DIR "icon.gif"

struct TextStyle {
  QString colour;
  QString backcolour;
  QString font;
  bool bold;
  bool italic;
  int size;
};

struct Extremum {
  float value;
  QString alarm;
  struct TextStyle textstyle;
};

struct Extrema {
  struct Extremum hi, lo, xhi, xlo;
};

struct MultiVal {
  int value;
  QString caption;
  QString alarm;
  struct TextStyle textstyle;
};

struct Box {
  QString caption;
  int row, col;
  int rowspan, colspan;
  int boxindex;
  struct TextStyle textstyle;
};

struct Label {
  QString caption;
  char src[20];
  int parentbox;
  int labelindex;
  char datumtype;
  int index;
  bool dialogup;
  int laststyle;
  struct TextStyle textstyle;
};

struct Number {
  char format[25];
  struct Extrema extrema;
  struct TextStyle textstyle;
};

struct Deriv {
  char format[25];
  struct Extrema extrema;
  struct TextStyle textstyle;
  int length;
  unsigned long tfactor;
  int first;
  int last;
  double* data;
};

struct Multi {
  int numwords;
  struct MultiVal words[MAX_MULTI_WORDS];
};

struct DateTime {
  char format[50];
  struct TextStyle textstyle;
};

struct CurDir {
  char format[50];
  struct TextStyle textstyle;
};

struct AlarmInfo {
  Label *lab;
  QString alarm;
  float val;
};

class QVBoxLayout;
class QHBoxLayout;
class QGridLayout;
class QLabel;
class QPushButton;
class QGroupBox;
class QSpacerItem;
class QDomElement;
class QDomNode;
class QPixmap;
class QComboBox;
class QFrame;
class QListBox;
class QMultiLineEdit;
class QThread;

class KstFile;
class AdamDom;

class PalImage
{
  public:
    PalImage();
    void TurnOn(QLabel *label);
    void TurnOff(QLabel *label);

    char framenum;
    char numframes;
    QPixmap *Images[4];
};

class DecomData
{
  public:
    DecomData();
    void setData(char*);
    int Status(void);
    double FrameLoss(void);
    double DataQuality(void);
    double DiskFree(void);
    char* DecomFile(void);

  private:
    unsigned long long int df;
    int status;
    int polarity;
    int decomUnlocks;
    double fs_bad;
    double dq_bad;
    char filename[256];
};

class DecomPoll : public QThread
{
  public:
    virtual void run();
};

class MainForm : public QMainWindow
{
  Q_OBJECT

  public:
    MainForm(QWidget* parent = 0, const char* name = 0, bool modal = FALSE,
        WFlags fl = 0, char *layoutfile = "layoutfile.pal");
    ~MainForm();

    QList<QGroupBox> QtBoxes;
    QList<QLabel> QtLabels;
    QList<QLabel> QtData;
    QLabel *ShowPicture;

  protected:
    QGridLayout* InfoLayout;
    QVBoxLayout* MainFormLayout;
    QList<QGridLayout> BoxLayout;
    QGridLayout* ContentLayout;

  private:
    QLabel* DecomState;
    QLabel* PalantirState;
    QLabel* LockState;
    QLabel* FrameLoss;
    QLabel* DataQuality;
    QLabel* DecomFile;
    QLabel* DiskFree;

    void WarningMessage(char* title, char* txt);
    void WarningMessage(char* title, QString txt);
    float QStringToFloat(QString str);
    int QStringToInt(QString str);
    bool QStringToBool(QString str);
    void GetXMLInfo(char *layoutfile);
    QString FindAttribute(char *attrib, char *tagname);
    void GetTextStyle(struct TextStyle *tstyle);
    void SetTextStyle(struct TextStyle *tstyle, int typebm, int bookmark);
    void GetExtrema(struct Extrema *ext, int bookmark);
    void SetExtrema(struct Extrema *ext, int bookmark);
    void GetWords(struct Multi *multi, int bookmark);
    void SetWords(struct Multi *multi, int bookmark);
    void GetStyle(QDomElement elem, struct TextStyle *tstyle);
    void TStyleInit(struct TextStyle *tstyle);
    void LabelInit(struct Label *lab);
    QPalette Palette(struct TextStyle tstyle);
    QFont Font(struct TextStyle tstyle);

    double GetSlope(struct Deriv *);

    QTimer *timer;

    QList<struct Box> BoxInfo;
    QList<struct Label> LabelInfo;
    QList<struct Number> NumberInfo;
    QList<struct Multi> MultiInfo;
    QList<struct DateTime> DateTimeInfo;
    QList<struct Deriv> DerivInfo;
    QList<struct CurDir> CurDirInfo;

    QString *CurFile;

    struct TextStyle ErrorStyle;
    struct TextStyle InfoCaptionStyle;
    struct TextStyle InfoComboStyle;
    struct TextStyle InfoDataStyle;
    struct TextStyle InfoButtonStyle;

    int firstpstyle;
    int numpstyles;
    char **pstyles;
    int firstpext;
    int numpexts;
    char **pexts;
    int firstpmulti;
    int numpmultis;
    char **pmultis;
    int firstpderiv;
    int numpderivs;
    char **pderivs;

    AdamDom *XMLInfo;
    DecomPoll *DecomPoller;

    bool startupDecomd;

    PalImage *Picture;
    QPixmap *Icon;
    KstFile *DataSource;

    QList<bool> DisabledIndex;

    char NoIncoming;
    bool NoIncomingOn;
    bool NoIncomingDialogUp;
    bool DialogsUp;
    int AlarmScroll;

    public slots:
      void UpdateData();
};

#endif
