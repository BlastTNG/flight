// ***************************************************
// *                                                 *
// *  Programmed by Adam Hincks                      *
// *                                                 *
// *  The program is badly organised in some ways,   *
// *  but it works fine.                             *
// *                                                 *
// ***************************************************

#include "kstfile.h"
#include "palantir.h"
#include "adamdom.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <sys/stat.h>
#include <qtimer.h>

// Define bookmark numbers
#define BM_RES_NUM     25
#define BM_RESERVE     9
#define BM_FIRST       0
#define BM_SECOND      1
#define BM_THIRD       2
#define BM_DEF_BOX     10
#define BM_DEF_LABEL   11
#define BM_DEF_DATUM   12
#define BM_DEF_WORD    13
#define BM_DEF_HI      14
#define BM_DEF_LO      15
#define BM_DEF_XHI     16
#define BM_DEF_XLO     17

#define MAX_N_DIALOGS  3



//***************************************************************************
//****
//****     CLASS PalImage -- does the cool animation of the palantir
//****
//***************************************************************************


//-------------------------------------------------------------
//
// PalImage: constructor
//
//-------------------------------------------------------------

PalImage::PalImage() {
  Images[0] = new QPixmap(PALANTIR_0_JPG);
  Images[1] = new QPixmap(PALANTIR_1_JPG);
  Images[2] = new QPixmap(PALANTIR_2_JPG);
  Images[3] = new QPixmap(PALANTIR_3_JPG);

  framenum = 0;
  numframes = 3;
}


//-------------------------------------------------------------
//
// TurnOn (public): Increment the frame number and show new
//      frame
//
//   *label: destination of new frame
//
//-------------------------------------------------------------

void PalImage::TurnOn(QLabel *label) {
  framenum++;
  if (framenum > numframes)
    framenum = 1;
  label->setPixmap(*Images[framenum]);
}

//-------------------------------------------------------------
//
// TurnOff (public): Show the blank palantir
//
//   *label: destination of frame
//
//-------------------------------------------------------------

void PalImage::TurnOff(QLabel *label) {
  framenum = 0;
  label->setPixmap(*Images[framenum]);
}




//***************************************************************************
//****
//****     CLASS MainForm: main control class
//****
//***************************************************************************




//-------------------------------------------------------------
//
// WarningMessage (private):  Pop-up Window
//
//   title: window title
//   txt: the warning message
//
//-------------------------------------------------------------

void MainForm::WarningMessage(char title[], char txt[]) {
  QMessageBox::information(this, title, txt, QMessageBox::Ok |
      QMessageBox::Default);
}

void MainForm::WarningMessage(char title[], QString txt) {
  QMessageBox::information(this, title, txt, QMessageBox::Ok |
      QMessageBox::Default);
}


//-------------------------------------------------------------
//
// QStringToInt, QStringToFloat etc.: user-friendly atoX
//
//   str: QString to be converted
//
//   Returns: string converted to int, float, or bool if
//      possible, otherwise returns 0 or false
//
//-------------------------------------------------------------

int MainForm::QStringToInt(QString str) {
  if (str == "")
    return 0;
  else
    return atoi(str);
}

float MainForm::QStringToFloat(QString str) {
  if (str == "")
    return 0;
  else
    return atof(str);
}

bool MainForm::QStringToBool(QString str) {
  if (str != "true" && str != "TRUE" && str != "True")
    return false;
  else
    return true;
}


//-------------------------------------------------------------
//
// TStyleInit (private): give default values to a textstyle,
//      in case none whatsoever are specified in .pal file
//
//   *tstyle: textstyle to write to
//
//-------------------------------------------------------------

void MainForm::TStyleInit(struct TextStyle *tstyle) {
  tstyle->colour = "#000000";
  tstyle->backcolour = "#dcdcdc";
  tstyle->font = "adobe-helvetica";
  tstyle->bold = false;
  tstyle->italic = false;
  tstyle->size = 8;
}


//-------------------------------------------------------------
//
// LabelInit (private): mainly to make sure that alarmenabled
//      and dialog up start with the right values
//
//   *lab: lab to write to
//
//-------------------------------------------------------------

void MainForm::LabelInit(struct Label *lab) {
  lab->caption = "-*-";
  strcpy(lab->src, "");
  lab->parentbox = lab->labelindex = lab->datumtype = lab->index = -1;
  lab->dialogup = false;
  lab->laststyle = -1;
}


//-------------------------------------------------------------
//
// FindAttribute (private): asks the XMLInfo object for an
//      attribute; displays a warning message if it is not
//      found
//
//   *attrib: the attribute to look for
//   *tagname: an string identifying the tag in case the
//      warning message is needed
//
//   Returns: the value of the attribute or "" if not found
//
//-------------------------------------------------------------

QString MainForm::FindAttribute(char *attrib, char *tagname) {
  char tmp[255];

  if (XMLInfo->GetAttribute(attrib) == "") {
    sprintf(tmp, "Tag %s missing attribute: %s:", tagname, attrib);
    WarningMessage("Parse Error", tmp);
    return "";
  } else
    return XMLInfo->GetAttribute(attrib);
}


//-------------------------------------------------------------
//
// GetTextStyle (private): gets any textstyle attributes from
//      the current XML tag
//
//   *tstyle: the textstyle to write to
//
//-------------------------------------------------------------

void MainForm::GetTextStyle(struct TextStyle *tstyle) {
  if (XMLInfo->GetAttribute("colour") != "")
    tstyle->colour = XMLInfo->GetAttribute("colour");
  if (XMLInfo->GetAttribute("backcolour") != "")
    tstyle->backcolour = XMLInfo->GetAttribute("backcolour");
  if (XMLInfo->GetAttribute("font") != "")
    tstyle->font = XMLInfo->GetAttribute("font");
  if (XMLInfo->GetAttribute("size") != "")
    tstyle->size = QStringToInt(XMLInfo->GetAttribute("size"));
  if (XMLInfo->GetAttribute("bold") != "")
    tstyle->bold = QStringToBool(XMLInfo->GetAttribute("bold"));
  if (XMLInfo->GetAttribute("italic") != "")
    tstyle->italic = QStringToBool(XMLInfo->GetAttribute("italic"));
}


//-------------------------------------------------------------
//
// SetTextStyle (private): initialises textstyle, looks for
//      default and predefined textstyles before getting any
//      textstyles from the current tag
//
//   *tstyle: the textstyle to write to
//   typebm: bookmark of the default textstyle to use (if -1,
//      use none)
//   bookmark: bookmark of current tag
//
//-------------------------------------------------------------

void MainForm::SetTextStyle(struct TextStyle *tstyle, int typebm,
    int bookmark) {
  char predef[35];
  int i;

  strcpy(predef, XMLInfo->GetAttribute("textstyle"));
  TStyleInit(tstyle);

  if (typebm != -1) {
    XMLInfo->GoBookMark(typebm);
    SetTextStyle(tstyle, -1, typebm);
  }

  for (i = 0; i < numpstyles; i++) {
    if (strcmp(predef, pstyles[i]) == 0) {
      XMLInfo->GoBookMark(i + firstpstyle);
      GetTextStyle(tstyle);
      break;
    }
  }

  XMLInfo->GoBookMark(bookmark);
  GetTextStyle(tstyle);
}


//-------------------------------------------------------------
//
// GetExtrema (private): get any extrema attributes located in
//      any extremum children (HI, LO, XHI, XLO) of the current
//      tag
//
//   *ext: the extrema to write to
//   bookmark: bookmark of current tag
//
//-------------------------------------------------------------

void MainForm::GetExtrema(struct Extrema *ext, int bookmark) {
  struct Extremum *currExtremum;
  int extbm;

  for (XMLInfo->GotoFirstChild(); !XMLInfo->NullEntry();
      XMLInfo->GotoNextSib()) {
    XMLInfo->SetBookMark(BM_RESERVE);
    if (XMLInfo->GetTagName() == "HI") {
      extbm = BM_DEF_HI;
      currExtremum = &(ext->hi);
    } else if (XMLInfo->GetTagName() == "LO") {
      extbm = BM_DEF_LO;
      currExtremum = &(ext->lo);
    } else if (XMLInfo->GetTagName() == "XHI") {
      extbm = BM_DEF_XHI;
      currExtremum = &(ext->xhi);
    } else if (XMLInfo->GetTagName() == "XLO") {
      extbm = BM_DEF_XLO;
      currExtremum = &(ext->xlo);
    } else
      currExtremum = NULL;

    if (currExtremum != NULL) {
      SetTextStyle(&(currExtremum->textstyle), extbm, BM_RESERVE);
      if (XMLInfo->GetAttribute("value") != "")
        currExtremum->value = QStringToFloat(XMLInfo->GetAttribute("value"));
    }
  }

  XMLInfo->GoBookMark(bookmark);
}


//-------------------------------------------------------------
//
// SetExtrema (private): looks for predefined extrema and then
//      gets extrema from the current tag's children
//
//   *ext: the extrema to write to
//   bookmark: bookmark of the current tag
//
//-------------------------------------------------------------

void MainForm::SetExtrema(struct Extrema *ext, int bookmark) {
  int i;
  char predef[35];
  strcpy(predef, XMLInfo->GetAttribute("extrema"));

  for (i = 0; i < numpexts; i++) {
    if (strcmp(predef, pexts[i]) == 0) {
      XMLInfo->GoBookMark(i + firstpext);
      GetExtrema(ext, i + firstpext);
      break;
    }
  }
  XMLInfo->GoBookMark(bookmark);
  GetExtrema(ext, bookmark);
}


//-------------------------------------------------------------
//
// GetWords (private): gets any attributes from any word tags
//      that are children of the current tag
//
//   *multi: the multi to write the words to
//   bookmark: bookmark of the current tag
//
//-------------------------------------------------------------

void MainForm::GetWords(struct Multi *multi, int bookmark) {
  multi->numwords = 0;
  for (XMLInfo->GotoFirstChild(); !XMLInfo->NullEntry();
      XMLInfo->GotoNextSib()) {
    XMLInfo->SetBookMark(BM_RESERVE);
    if (XMLInfo->GetAttribute("value") != "")
      multi->words[multi->numwords].value =
        QStringToInt(XMLInfo->GetAttribute("value"));
    if (XMLInfo->GetAttribute("caption") != "")
      multi->words[multi->numwords].caption = XMLInfo->GetAttribute("caption");
    SetTextStyle(&(multi->words[multi->numwords].textstyle), BM_DEF_WORD,
        BM_RESERVE);
    multi->numwords++;
  }
  XMLInfo->GoBookMark(bookmark);
}


//-------------------------------------------------------------
//
// SetWords (private): looks for predefined multis and then
//      looks for any child word tags of the current word
//
//   *multi: the multi to write to
//   bookmark: bookmark of the current tag
//
//-------------------------------------------------------------

void MainForm::SetWords(struct Multi *multi, int bookmark) {
  int i;
  char predef[35];
  strcpy(predef, XMLInfo->GetAttribute("words"));

  for (i = 0; i < numpmultis; i++) {
    if (strcmp(predef, pmultis[i]) == 0) {
      XMLInfo->GoBookMark(i + firstpmulti);
      GetWords(multi, i + firstpmulti);
      break;
    }
  }

  XMLInfo->GoBookMark(bookmark);
  GetWords(multi, bookmark);
}

//-------------------------------------------------------------
//
// GetXMLInfo (private): parses out the information from the
//      XML file.  Note that here as in functions above, the
//      AdamDom class is used to read the XML file.
//
//   *layoutfile: file name of the XML file
//
//-------------------------------------------------------------

void MainForm::GetXMLInfo(char *layoutfile) {
  int i, j, k;
  struct Box *currBox;
  struct Label *currLabel;
  struct Number *currNumber;
  struct Multi *currMulti;
  struct Deriv *currDeriv;
  struct DateTime *currDateTime;
  struct CurDir *currCurDir;
  QString *currCurFile;
  char tmp[50];

  // Load XML file
  if (!XMLInfo->LoadXML(layoutfile))
    WarningMessage("File Error", "Could not read xml file.");

  // Create bookmarks
  XMLInfo->AddBookMarks(BM_RES_NUM);

  // The following code prepares references to predefined textstyles,
  // extrema, and multis.  An array of strings carrying the names
  // of the predefines is created, each associated with a bookmark
  // to that tag for quick access later
  //   pstyles, pexts, pmultis -- arrays (p = predefined)
  //   numpstyles etc. -- number of predefines
  //   firstpstyle etc. -- the bookmark number of the first predefine
  numpstyles = i = XMLInfo->CountEntries(".SETTINGS.TEXTSTYLE", false);
  XMLInfo->AddBookMarks(i);
  pstyles = (char **)malloc(i * sizeof(char *));
  firstpstyle = BM_RES_NUM;

  numpexts = i = XMLInfo->CountEntries(".SETTINGS.EXTREMA", false);
  XMLInfo->AddBookMarks(i);
  pexts = (char **)malloc(i * sizeof(char *));
  firstpext = firstpstyle + numpstyles;

  numpmultis = i = XMLInfo->CountEntries(".SETTINGS.MULTI", false);
  XMLInfo->AddBookMarks(i);
  pmultis = (char **)malloc(i * sizeof(char *));
  firstpmulti = firstpext + numpexts;

  numpderivs = i = XMLInfo->CountEntries(".SETTINGS.DERIV", false);
  XMLInfo->AddBookMarks(i);
  pderivs = (char **)malloc(i * sizeof(char *));
  firstpderiv = firstpmulti + numpmultis;

  XMLInfo->GotoEntry(".SETTINGS", 0, false);
  i = 0;
  j = 0;
  k = 0;
  for (XMLInfo->GotoFirstChild(); !XMLInfo->NullEntry();
      XMLInfo->GotoNextSib()) {
    strcpy(tmp, XMLInfo->GetTagName());
    if (XMLInfo->GetTagName() == "TEXTSTYLE") {
      XMLInfo->SetBookMark(firstpstyle + i);
      pstyles[i] = (char *)malloc(35 * sizeof(char));
      strcpy(pstyles[i], FindAttribute("name", "SETTINGS.TEXTSTYLE"));
      i++;
    }
    if (XMLInfo->GetTagName() == "EXTREMA") {
      XMLInfo->SetBookMark(firstpext + j);
      pexts[j] = (char *)malloc(35 * sizeof(char));
      strcpy(pexts[j], FindAttribute("name", "SETTINGS.EXTREMA"));
      j++;
    }
    if (XMLInfo->GetTagName() == "MULTI") {
      XMLInfo->SetBookMark(firstpmulti + k);
      pmultis[k] = (char *)malloc(35 * sizeof(char));
      strcpy(pmultis[k], FindAttribute("name", "SETTINGS.MULTI"));
      k++;
    }
    if (XMLInfo->GetTagName() == "DERIV") {
      XMLInfo->SetBookMark(firstpmulti + k);
      pderivs[k] = (char *)malloc(35 * sizeof(char));
      strcpy(pderivs[k], FindAttribute("name", "SETTINGS.DERIV"));
      k++;
    }
  }

  // Read in textstyles for the look of the tool bar on the bottom of
  // Palantir (as well as the style to use when there is a read error)
  TStyleInit(&ErrorStyle);
  TStyleInit(&InfoCaptionStyle);
  TStyleInit(&InfoDataStyle);
  TStyleInit(&InfoComboStyle);
  TStyleInit(&InfoButtonStyle);

  if (XMLInfo->GotoEntry(".DEFAULTS.TEXTSTYLE.DATUM.ERROR", 0, false)) {
    XMLInfo->SetBookMark(BM_FIRST);
    SetTextStyle(&ErrorStyle, -1, BM_FIRST);
  }
  if (XMLInfo->GotoEntry(".DEFAULTS.TEXTSTYLE.INFOCAPTION", 0, false)) {
    XMLInfo->SetBookMark(BM_FIRST);
    SetTextStyle(&InfoCaptionStyle, -1, BM_FIRST);
  }
  if (XMLInfo->GotoEntry(".DEFAULTS.TEXTSTYLE.INFODATA", 0, false)) {
    XMLInfo->SetBookMark(BM_FIRST);
    SetTextStyle(&InfoDataStyle, -1, BM_FIRST);
  }
  if (XMLInfo->GotoEntry(".DEFAULTS.TEXTSTYLE.INFOCOMBO", 0, false)) {
    XMLInfo->SetBookMark(BM_FIRST);
    SetTextStyle(&InfoComboStyle, -1, BM_FIRST);
  }
  if (XMLInfo->GotoEntry(".DEFAULTS.TEXTSTYLE.INFOBUTTON", 0, false)) {
    XMLInfo->SetBookMark(BM_FIRST);
    SetTextStyle(&InfoButtonStyle, -1, BM_FIRST);
  }

  // Bookmark the default textstyles
  XMLInfo->GotoEntry(".DEFAULTS.TEXTSTYLE.BOX", 0, false);
  XMLInfo->SetBookMark(BM_DEF_BOX);
  XMLInfo->GotoEntry(".DEFAULTS.TEXTSTYLE.LABEL", 0, false);
  XMLInfo->SetBookMark(BM_DEF_LABEL);
  XMLInfo->GotoEntry(".DEFAULTS.TEXTSTYLE.DATUM", 0, false);
  XMLInfo->SetBookMark(BM_DEF_DATUM);
  XMLInfo->GotoEntry(".DEFAULTS.TEXTSTYLE.DATUM.WORD", 0, false);
  XMLInfo->SetBookMark(BM_DEF_WORD);
  XMLInfo->GotoEntry(".DEFAULTS.TEXTSTYLE.DATUM.HI", 0, false);
  XMLInfo->SetBookMark(BM_DEF_HI);
  XMLInfo->GotoEntry(".DEFAULTS.TEXTSTYLE.DATUM.LO", 0, false);
  XMLInfo->SetBookMark(BM_DEF_LO);
  XMLInfo->GotoEntry(".DEFAULTS.TEXTSTYLE.DATUM.XHI", 0, false);
  XMLInfo->SetBookMark(BM_DEF_XHI);
  XMLInfo->GotoEntry(".DEFAULTS.TEXTSTYLE.DATUM.XLO", 0, false);
  XMLInfo->SetBookMark(BM_DEF_XLO);

  // Find all the BOX tags
  for (XMLInfo->GotoEntry(".BOX", 0, false); !XMLInfo->NullEntry();
      XMLInfo->GotoNextSib()) {
    BoxInfo.append(new Box);
    currBox = BoxInfo.current();
    XMLInfo->SetBookMark(BM_FIRST);

    currBox->caption = FindAttribute("caption", "BOX");
    currBox->row = QStringToInt(FindAttribute("row", "BOX"));
    currBox->col = QStringToInt(FindAttribute("col", "BOX"))-1;
    currBox->rowspan = QStringToInt(XMLInfo->GetAttribute("rowspan"));
    currBox->colspan = QStringToInt(XMLInfo->GetAttribute("colspan"));

    SetTextStyle(&(currBox->textstyle), BM_DEF_BOX, BM_FIRST);

    // Find all the BOX's children
    for (XMLInfo->GotoFirstChild(); !XMLInfo->NullEntry();
        XMLInfo->GotoNextSib()) {
      XMLInfo->SetBookMark(BM_SECOND);
      LabelInfo.append(new Label);
      currLabel = LabelInfo.current();
      LabelInit(currLabel);
      currLabel->caption = FindAttribute("caption", "BOX.NUMBER");
      currLabel->parentbox = BoxInfo.count() - 1;
      SetTextStyle(&(currLabel->textstyle), BM_DEF_LABEL, BM_SECOND);
      strcpy(tmp, XMLInfo->GetAttribute("caption"));

      // NUMBER fields
      if (XMLInfo->GetTagName() == "NUMBER") {
        if (XMLInfo->GotoEntry(".DATUM", 0, true)) {
          XMLInfo->SetBookMark(BM_THIRD);
          NumberInfo.append(new Number);
          currNumber = NumberInfo.current();
          currLabel->datumtype = NUMBER;
          currLabel->index = NumberInfo.count() - 1;
          strcpy(currLabel->src, FindAttribute("src", "BOX.NUMBER.DATUM"));
          strcpy(currNumber->format, FindAttribute("format",
                "BOX.NUMBER.DATUM"));
          SetTextStyle(&(currNumber->textstyle), BM_DEF_DATUM, BM_THIRD);
          SetExtrema(&(currNumber->extrema), BM_THIRD);
        }
      } else if (XMLInfo->GetTagName() == "MULTI") {
        if (XMLInfo->GotoEntry(".DATUM", 0, true)) {
          XMLInfo->SetBookMark(BM_THIRD);
          MultiInfo.append(new Multi);
          currMulti = MultiInfo.current();

          currLabel->datumtype = MULTI;
          currLabel->index = MultiInfo.count() - 1;
          strcpy(currLabel->src, FindAttribute("src", "BOX.MULTI.DATUM"));
          SetWords(currMulti, BM_THIRD);
        }
      } else if (XMLInfo->GetTagName() == "DATETIME") {
        if (XMLInfo->GotoEntry(".DATUM", 0, true)) {
          XMLInfo->SetBookMark(BM_THIRD);
          DateTimeInfo.append(new DateTime);
          currDateTime = DateTimeInfo.current();

          currLabel->datumtype = DATETIME;
          currLabel->index = DateTimeInfo.count() - 1;
          strcpy(currLabel->src, FindAttribute("src", "BOX.DATETIME.DATUM"));
          strcpy(currDateTime->format, FindAttribute("format",
                "BOX.DATETIME.DATUM"));
          SetTextStyle(&(currDateTime->textstyle), BM_DEF_DATUM, BM_THIRD);
        }
      } else if (XMLInfo->GetTagName() == "CURDIR") {
        if (XMLInfo->GotoEntry(".DATUM", 0, true)) {
          XMLInfo->SetBookMark(BM_THIRD);
          CurDirInfo.append(new CurDir);
          currCurDir = CurDirInfo.current();

          currLabel->datumtype = CURDIR;
          currLabel->index = CurDirInfo.count() - 1;
          SetTextStyle(&(currCurDir->textstyle), BM_DEF_DATUM, BM_THIRD);
        }
      } else if (XMLInfo->GetTagName() == "DERIV") {
        if (XMLInfo->GotoEntry(".DATUM", 0, true)) {
          XMLInfo->SetBookMark(BM_THIRD);
          DerivInfo.append(new Deriv);
          currDeriv = DerivInfo.current();

          currLabel->datumtype = DERIV;
          currLabel->index = DerivInfo.count() - 1;
          strcpy(currLabel->src, FindAttribute("src", "BOX.DERIV.DATUM"));
          currDeriv->length = QStringToInt(FindAttribute("length",
                "BOX.DERIV.DATUM"));
          if (currDeriv->length <= 1)
            currDeriv->length = 2;
          currDeriv->data = (double *)malloc(currDeriv->length *
              sizeof(double));
          currDeriv->first = currDeriv->last = -1;
          strcpy(tmp, FindAttribute("tunit", "BOX.DERIV.DATUM"));
          if ((currDeriv->tfactor = atoi(tmp)) == 0) {
            if (strcmp("ms", tmp) == 0)      // millisecond
              currDeriv->tfactor = 1UL;
            else if (strcmp("m", tmp) == 0)  // minute
              currDeriv->tfactor = 60000UL;
            else if (strcmp("h", tmp) == 0)  // hour
              currDeriv->tfactor = 3600000UL;
            else if (strcmp("d", tmp) == 0)  // day
              currDeriv->tfactor = 86400000UL;
            else                             // default to second
              currDeriv->tfactor = 1000UL;
          }
          strcpy(currDeriv->format, FindAttribute("format",
                "BOX.NUMBER.DATUM"));
          SetTextStyle(&(currDeriv->textstyle), BM_DEF_DATUM, BM_THIRD);
          SetExtrema(&(currDeriv->extrema), BM_THIRD);
        }
      }

      // Return to the BM_SECOND tag so that the "for" loop will increment
      // properly (when it calls GotoNextSib())
      XMLInfo->GoBookMark(BM_SECOND);
    }
    // Return to the BM_FIRST tag
    XMLInfo->GoBookMark(BM_FIRST);
  }

  // Read in the .cur file names
  for (XMLInfo->GotoEntry(".SETTINGS.CURVEFILE", 0, false);
      !XMLInfo->NullEntry(); XMLInfo->GotoNextSib()) {
    if (XMLInfo->GetTagName() == "CURVEFILE") {
      CurveFiles.append(new QString);
      currCurFile = CurveFiles.current();
      *currCurFile = FindAttribute("name", "SETTINGS.CURVEFILE");
    }
  }
}


//-------------------------------------------------------------
//
// Palette (private): prepare a QPalette, using the given
//      style
//
//   tstyle: styles to use
//
//   Returns: the constructed QPalette
//
//-------------------------------------------------------------

QPalette MainForm::Palette(struct TextStyle tstyle) {
  QPalette pal;
  QColorGroup cg;

  cg.setColor(QColorGroup::Button, QColor(220, 220, 220));
  cg.setColor(QColorGroup::Light, QColor(255, 255, 255));
  cg.setColor(QColorGroup::Midlight, QColor(237, 237, 237));
  cg.setColor(QColorGroup::Dark, QColor(110, 110, 110));
  cg.setColor(QColorGroup::Mid, QColor(146, 146, 146));
  cg.setColor(QColorGroup::Text, QColor(0, 0, 0));
  cg.setColor(QColorGroup::BrightText, QColor(255, 255, 255));
  cg.setColor(QColorGroup::ButtonText, QColor(0, 0, 0));
  cg.setColor(QColorGroup::Base, QColor(255, 255, 255));
  cg.setColor(QColorGroup::Shadow, QColor(0, 0, 0));
  cg.setColor(QColorGroup::Highlight, QColor("black"));
  cg.setColor(QColorGroup::HighlightedText, QColor("white"));
  cg.setColor(QColorGroup::Foreground, QColor(tstyle.colour));
  cg.setColor(QColorGroup::Background, QColor(tstyle.backcolour));
  pal.setActive(cg);
  pal.setInactive (cg);
  pal.setDisabled(cg);

  return pal;
}


//-------------------------------------------------------------
//
// Font (private): prepare a QFont, using the given style
//
//   tstyle: styles to use
//
//   Returns: the constructed QFont
//
//-------------------------------------------------------------

QFont MainForm::Font(struct TextStyle tstyle) {
  QFont font;
  font.setFamily(tstyle.font);
  font.setPointSize(tstyle.size);
  font.setBold(tstyle.bold);
  font.setItalic(tstyle.italic);

  return font;
}

//-------------------------------------------------------------
//
// GetSlope: performs linear regression on the Deriv's buffer to
//     obtain a slope.  Since we're only calculating the slope
//     and not the intercept, and we have no sigmas, we can pare
//     down the canonical linreg routine a fair bit...
//
//-------------------------------------------------------------

double MainForm::GetSlope(struct Deriv *currDeriv) {
  int length = currDeriv->length;
  int first = currDeriv->first;
  int last = currDeriv->last;
  int i, j;

  double b = 0;     // the slope
  double f, t, v = 0;  // scratch space

  if (first == last) // fifo full, read all data
    j = length;
  else // fifo partially full, read only up to last;
    j = last;

  f = (j - 1) / 2.;  // = sum(i, i=0..j) / j (since the x's are equally spaced)

  for (i = 0; i < j; ++i) {
    t = ((i + length - first) % length) - f;  // true fifo index
    v += t * t;
    b += t * currDeriv->data[i];
  }

  b /= v;

  // Calculated slope is wrt the UPDATETIME period (which is specified in
  // milliseconds), so "first" divide by UPDATETIME to get wrt milliseconds and
  // then multiply by the user specified time factor

  return b * currDeriv->tfactor / UPDATETIME;
}

//-------------------------------------------------------------
//
// UpdateData (slot): use the KstFile class to read in new
//      frames from disk.  Fired by timer.
//
//-------------------------------------------------------------

void MainForm::UpdateData() {
  double indata[20];
  char displayer[80];
  char i, j;
  struct Label *currLabel;
  struct Number *currNumber;
  struct Multi *currMulti;
  struct Deriv *currDeriv;
  struct DateTime *currDateTime;
  struct CurDir *currCurDir;
  QLabel *currQtLabel;
  time_t timetmp;
  struct tm *currTime;
  char tmp[255];
  int updating;
  FILE *curf;

  if (DataSource->update()) {
    updating = 1;
    Picture->TurnOn(ShowPicture);
  } else {
    // Blank palantir
    Picture->TurnOff(ShowPicture);
    updating = 0;
    if (NoIncomingOn) {
      if (++NoIncoming == 3) {
        NoIncoming = 0;
      }
    }
  }

  // Loop through all the data fields we need to read
  for (currLabel = LabelInfo.first(); currLabel != NULL;
      currLabel = LabelInfo.next()) {
    switch (currLabel->datumtype) {
      case NUMBER:
        currNumber = NumberInfo.at(currLabel->index);
        currQtLabel = QtData.at(currLabel->labelindex);
        // Read in from disk
        if (DataSource->readField(indata, currLabel->src,
              DataSource->numFrames() - 2, 1) == 0) {
          if (currLabel->laststyle != 1) {
            currQtLabel->setPalette(Palette(ErrorStyle));
            currQtLabel->setFont(Font(ErrorStyle));
            currQtLabel->setText(tr("NAN"));
            currLabel->laststyle = 1;
          }
        } else {
          // Check to see if value lies outside of any extremum
          if (*indata >= currNumber->extrema.xhi.value) {
            if (currLabel->laststyle != 2) {
              currQtLabel->setPalette(Palette(
                    currNumber->extrema.xhi.textstyle));
              currQtLabel->setFont(Font(currNumber->extrema.xhi.textstyle));
              currLabel->laststyle = 2;
            }
          } else if (*indata >= currNumber->extrema.hi.value) {
            if (currLabel->laststyle != 3) {
              currQtLabel->setPalette(Palette(
                    currNumber->extrema.hi.textstyle));
              currQtLabel->setFont(Font(currNumber->extrema.hi.textstyle));
              currLabel->laststyle = 3;
            }
          } else if (*indata <= currNumber->extrema.xlo.value) {
            if (currLabel->laststyle != 4) {
              currQtLabel->setPalette(Palette(
                    currNumber->extrema.xlo.textstyle));
              currQtLabel->setFont(Font(currNumber->extrema.xlo.textstyle));
              currLabel->laststyle = 4;
            }
          } else if (*indata <= currNumber->extrema.lo.value) {
            if (currLabel->laststyle != 5) {
              currQtLabel->setPalette(Palette(
                    currNumber->extrema.lo.textstyle));
              currQtLabel->setFont(Font(currNumber->extrema.lo.textstyle));
              currLabel->laststyle = 5;
            }
          } else {
            if (currLabel->laststyle != 0) {
              currQtLabel->setPalette(Palette(currNumber->textstyle));
              currQtLabel->setFont(Font(currNumber->textstyle));
              currLabel->laststyle = 0;
            }
          }
          sprintf(displayer, currNumber->format, *indata);
          if (strlen(displayer)==1) {
            displayer[1] = ' ';
            displayer[2] = '\0';
          }
          currQtLabel->setText(tr(displayer));
        }
        break;
      case MULTI:
        currMulti = MultiInfo.at(currLabel->index);
        currQtLabel = QtData.at(currLabel->labelindex);
        // Read in value from disk
        if (DataSource->readField(indata, currLabel->src,
              DataSource->numFrames() - 1, 1) == 0) {
          if (currLabel->laststyle != 1001) {
            currQtLabel->setPalette(Palette(ErrorStyle));
            currQtLabel->setFont(Font(ErrorStyle));
            currQtLabel->setText(tr("bad src"));
            currLabel->laststyle = 1001;
          }
        } else {
          // Determine which word to display based on the value
          j = 0;
          for (i = 0; i <= currMulti->numwords; i++)
            if ((char)(*indata) == currMulti->words[i].value) {
              if (currLabel->laststyle != i) {
                currQtLabel->setPalette(Palette(
                      currMulti->words[i].textstyle));
                currQtLabel->setFont(Font(currMulti->words[i].textstyle));
                currQtLabel->setText(tr(currMulti->words[i].caption));
                currLabel->laststyle = i;
              }
              j = 1;
              break;
            }
          if (!j) {
            if (currLabel->laststyle != 1002) {
              currQtLabel->setPalette(Palette(ErrorStyle));
              currQtLabel->setFont(Font(ErrorStyle));
              currQtLabel->setText(tr("NAN"));
              currLabel->laststyle = 1002;
            }
          } else
            j = 0;
        }
        break;
      case DATETIME:
        currDateTime = DateTimeInfo.at(currLabel->index);
        currQtLabel = QtData.at(currLabel->labelindex);
        // Read in value from disk
        if (DataSource->readField(indata, currLabel->src,
              DataSource->numFrames() - 1, 1) == 0) {
          if (currLabel->laststyle != 1) {
            currQtLabel->setPalette(Palette(ErrorStyle));
            currQtLabel->setFont(Font(ErrorStyle));
            currQtLabel->setText(tr("bad src"));
            currLabel->laststyle = 1;
          }
        } else {
          timetmp = (time_t)*indata;
          currTime = gmtime(&timetmp);
          if (currLabel->laststyle != 0) {
            currQtLabel->setPalette(Palette(currDateTime->textstyle));
            currQtLabel->setFont(Font(currDateTime->textstyle));
            currLabel->laststyle = 0;
          }
          strftime(tmp, 255, currDateTime->format, currTime);
          currQtLabel->setText(tr(tmp));
        }
        break;
      case CURDIR:
        currCurDir = CurDirInfo.at(currLabel->index);
        currQtLabel = QtData.at(currLabel->labelindex);
        strncpy (tmp, DataSource->fileName(), 254);
        if ((curf = fopen(tmp, "r")) == 0){
          if (currLabel->laststyle != 1) {
            currQtLabel->setPalette(Palette(ErrorStyle));
            currQtLabel->setFont(Font(ErrorStyle));
            currQtLabel->setText(tr("bad src"));
            currLabel->laststyle = 1;
          }
        } else {
          if (currLabel->laststyle != 0) {
            currQtLabel->setPalette(Palette(currCurDir->textstyle));
            currQtLabel->setFont(Font(currCurDir->textstyle));
            currLabel->laststyle = 0;
          }
          fscanf(curf, "%s", tmp);
          currQtLabel->setText(tr(basename(tmp)));
          fclose(curf);
        }
        break;
      case DERIV:
        currDeriv = DerivInfo.at(currLabel->index);
        currQtLabel = QtData.at(currLabel->labelindex);
        // Read in from disk
        if (DataSource->readField(indata, currLabel->src,
              DataSource->numFrames() - 1, 1) == 0) {
          if (currLabel->laststyle != 1) {
            currQtLabel->setPalette(Palette(ErrorStyle));
            currQtLabel->setFont(Font(ErrorStyle));
            currQtLabel->setText(tr("NAN"));
            currLabel->laststyle = 1;
          }
        } else if (updating) {
          // Push value onto the fifo
          int dlength = currDeriv->length;
          int emptyfifo = 0;
          if (currDeriv->first == -1) {
            // Fifo is empty
            emptyfifo = 1;
            currDeriv->data[0] = *indata;
            currDeriv->first = 0;
            currDeriv->last = 1;
            if (currLabel->laststyle != 2001) {
              currQtLabel->setPalette(Palette(ErrorStyle));
              currQtLabel->setFont(Font(ErrorStyle));
              currQtLabel->setText(tr("No Data"));
              currLabel->laststyle = 2001;
            }
          } else if (currDeriv->first == currDeriv->last) {
            // Fifo is full
            currDeriv->data[currDeriv->last] = *indata;
            currDeriv->last = (currDeriv->last + 1) % dlength;
            currDeriv->first = (currDeriv->first + 1) % dlength;
          } else {
            // Fifo is partially full
            currDeriv->data[currDeriv->last] = *indata;
            currDeriv->last = (currDeriv->last + 1) % dlength;
            dlength = currDeriv->last - currDeriv->first;
          }

          // If the fifo is empty, do nothing more
          if (emptyfifo)
            break;

          // Calculate slope of the data
          *indata = GetSlope(currDeriv);

          // Check to see if value lies outside of any extremum
          if (*indata >= currDeriv->extrema.xhi.value) {
            if (currLabel->laststyle != 2) {
              currQtLabel->setPalette(Palette(
                    currDeriv->extrema.xhi.textstyle));
              currQtLabel->setFont(Font(currDeriv->extrema.xhi.textstyle));
              currLabel->laststyle = 2;
            }
          } else if (*indata >= currDeriv->extrema.hi.value) {
            if (currLabel->laststyle != 3) {
              currQtLabel->setPalette(Palette(
                    currDeriv->extrema.hi.textstyle));
              currQtLabel->setFont(Font(currDeriv->extrema.hi.textstyle));
              currLabel->laststyle = 3;
            }
          } else if (*indata <= currDeriv->extrema.xlo.value) {
            if (currLabel->laststyle != 4) {
              currQtLabel->setPalette(Palette(
                    currDeriv->extrema.xlo.textstyle));
              currQtLabel->setFont(Font(currDeriv->extrema.xlo.textstyle));
              currLabel->laststyle = 4;
            }
          } else if (*indata <= currDeriv->extrema.lo.value) {
            if (currLabel->laststyle != 5) {
              currQtLabel->setPalette(Palette(
                    currDeriv->extrema.lo.textstyle));
              currQtLabel->setFont(Font(currDeriv->extrema.lo.textstyle));
              currLabel->laststyle = 5;
            }
          } else {
            if (currLabel->laststyle != 0) {
              currQtLabel->setPalette(Palette(currDeriv->textstyle));
              currQtLabel->setFont(Font(currDeriv->textstyle));
              currLabel->laststyle = 0;
            }
          }
          sprintf(displayer, currDeriv->format, *indata);
          currQtLabel->setText(tr(displayer));
        }
        break;
    }
  }
}



//-------------------------------------------------------------
//
// MainForm: constructor
//
//-------------------------------------------------------------

MainForm::MainForm(QWidget* parent,  const char* name, bool modal, WFlags fl,
    char *layoutfile) : QDialog(parent, name, modal, fl)
{
  char tmp[255];
  int row, i;
  int bytecount;
  int max_row=0, min_col = 10;

  struct Box *currQtBox;
  struct Label *currLabel;
  struct Number *currNumber;
  struct Multi *currMulti;
  struct Deriv *currDeriv;
  struct Extrema *currExtrema;
  struct TextStyle *currStyle;
  struct TextStyle tstyle = {"#000000", "#DCDCDC", "adobe-helvetica", false,
    true, 8};

  QGroupBox *currQtBoxes;
  QLabel *currQtPlaceHolder;
  QLabel *currQtLabels;
  QLabel *currQtData;
  QGridLayout *currQtBoxLayout;
  QSpacerItem *currSpacer;
  QString *currCurveFile;

  QFont font;

  XMLInfo = new AdamDom();
  GetXMLInfo(layoutfile);

  Icon = new QPixmap(ICON);

  if (!name)
    setName("MainForm");
  setCaption(tr("Palantir -")+layoutfile);
  setIcon(*Icon);

  ContentLayout = new QGridLayout;
  ContentLayout->setSpacing(2);
  ContentLayout->setMargin(0);

  // Create the screen using the info specified in the layout.pal file
  if (BoxInfo.isEmpty()) {
    WarningMessage("Error", "No boxes to display");
  } else {
    for (currQtBox = BoxInfo.first(); currQtBox != NULL;
        currQtBox = BoxInfo.next()) {

      // Create box
      QtBoxes.append(new QGroupBox(this, "Box"));
      currQtBoxes = QtBoxes.current();

      currQtBoxes->setTitle(tr(currQtBox->caption));
      currQtBoxes->setColumnLayout(0, Qt::Vertical);

      currQtBoxes->setPalette(Palette(currQtBox->textstyle));
      currQtBoxes->setFont(Font(currQtBox->textstyle));

      BoxLayout.append(new QGridLayout(currQtBoxes->layout()));
      currQtBoxLayout = BoxLayout.current();

      currQtBoxLayout->setAlignment(Qt::AlignTop);

      row = 0;
      for (currLabel = LabelInfo.first(); currLabel != NULL;
          currLabel = LabelInfo.next()) {
        // Create datum line inside current box
        if (currLabel->parentbox == BoxInfo.at()) {
          row ++;
          QtLabels.append(new QLabel(currQtBoxes, "Label"));
          currQtLabels = QtLabels.current();

          currQtLabels->setText(tr(currLabel->caption));
          currQtLabels->setPalette(Palette(currLabel->textstyle));
          currQtLabels->setFont(Font(currLabel->textstyle));

          if (currLabel->datumtype != -1) {
            QtData.append(new QLabel(currQtBoxes, "Label"));
            currQtData = QtData.current();
            currLabel->labelindex = QtData.at();
            currQtData->setText(tr("..."));
            if (currLabel->datumtype == NUMBER)
              currNumber = NumberInfo.at(currLabel->index);
            if (currLabel->datumtype == MULTI)
              currMulti = MultiInfo.at(currLabel->index);
            if (currLabel->datumtype == DERIV)
              currDeriv = DerivInfo.at(currLabel->index);
            currQtData->setPalette(Palette(tstyle));
            currQtData->setFont(Font(tstyle));
            currQtBoxLayout->addWidget(currQtData, row, 2);
          }
          currQtBoxLayout->addWidget(currQtLabels, row, 0);
        }
      }

      ContentLayout->addMultiCellWidget(currQtBoxes, currQtBox->row,
          currQtBox->row + currQtBox->rowspan,
          currQtBox->col,
          currQtBox->col + currQtBox->colspan);
      if (currQtBox->row + currQtBox->rowspan > max_row)
        max_row = currQtBox->row + currQtBox->rowspan;
      if (currQtBox->col < min_col) min_col = currQtBox->col;
    }
  }

  ShowPicture = new QLabel(this, "ShowPicture");
  ShowPicture->setScaledContents(false);
  ShowPicture->setFrameStyle(QFrame::Box|QFrame::Sunken);
  ShowPicture->setAlignment(Qt::AlignCenter);
  ShowPicture->setPaletteBackgroundColor(QColor("black"));
  ContentLayout->addMultiCellWidget(ShowPicture,max_row,
                                    max_row, min_col,min_col);

  if (CurveFiles.isEmpty())
    WarningMessage("Error",
        "You have specified NO .cur files in your layout file.");

  MainFormSpacer = new QSpacerItem(5, 5, QSizePolicy::Fixed,
      QSizePolicy::Fixed);

  MainFormLayout = new QVBoxLayout(this);
  MainFormLayout->setSpacing(2);
  MainFormLayout->setMargin(2);
  MainFormLayout->addLayout(ContentLayout);
  MainFormLayout->addItem(MainFormSpacer);
  //MainFormLayout->addWidget(ShowPicture);

  timer = new QTimer();
  NoIncoming = 0;
  NoIncomingOn = true;
  NoIncomingDialogUp = false;

  // Slots
  connect(timer, SIGNAL(timeout()), this, SLOT(UpdateData()));
  // Update palantir every UPDATETIME milliseconds
  timer->start(UPDATETIME, false);

  Picture = new PalImage();
  Picture->TurnOff(ShowPicture);

  if (!CurveFiles.isEmpty()) {
    currCurveFile = CurveFiles.first();
    strcpy(tmp, *currCurveFile);
  } else
    strcpy(tmp, '\0');

  // Initialise KstFile object
  DataSource = new KstFile(tmp, UNKNOWN);
}

MainForm::~MainForm()
{
  // No need to delete child widgets, Qt does it all for us
}


//***************************************************************************
//****
//****     Main()
//****
//***************************************************************************


void usage() {
  printf("\npalantir [layout file]\n\n");
  printf("Default layout file: " DEF_LAYOUTFILE "\n");
  printf("  [layout file] -> specify layout file\n\n");
}

int main(int argc, char* argv[]) {
  QApplication app(argc, argv);
  char layoutfile[25];

  // Parse out command line
  if (argc > 2) {
    usage();
    exit(1);
  }
  if (argc==2) {
    if (argv[1][0] == '-') {
      usage();
      exit(1);
    } else {
      strcpy(layoutfile, argv[1]);
    }
  } else {
    strcpy(layoutfile, DEF_LAYOUTFILE);
  }
  MainForm palantir(0, "palantir", true, 0, &layoutfile[0]);

  // Hand over control to Qt
  app.setMainWidget(&palantir);
  int ret = palantir.exec();

  return ret;
}
