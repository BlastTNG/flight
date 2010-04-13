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

// **********************************************************
// *  Programmed by Adam Hincks et al.                      *
// **********************************************************

//***************************************************************************
//****     CLASS AdamDom -- uses Qt's QDom class.  Can read in an XML file
//****                      and return information from it
//****     GLOSSARY
//****       DOM: document object model -- represents data in a type of
//****            hierchical tree.
//****       node: a joint on the DOM tree.  Nodes can have parents,
//****            children, and siblings
//****       element: a subtype of node.  An element is information contained
//****            in a tag
//****       tag: in the XML file, the stuff inside angled brackets
//****            (< . . .>)
//****       tag name: the capitalised name that begins a tag
//****            (<TAGNAME . . .>)
//****       attribute: information inside a tag -- e.g.
//****            <TAG attrib1="attrib">
//***************************************************************************

#include <stdio.h>
#include <stdlib.h>

#include "adamdom.h"
//-------------------------------------------------------------
// AdamDom: constructor
//-------------------------------------------------------------
AdamDom::AdamDom() {
  layout = new QDomDocument("layoutdoc");
}

AdamDom::AdamDom(char *filename) {
  layout = new QDomDocument("layoutdoc");

  LoadXML(filename);
}

//-------------------------------------------------------------
// LoadXML: uses QDom to parse an XML file into a DOM object.
//      "layout" is of class QDomDocument
//   *filename: XML file
//   Returns: true if successful, false if unsuccessful
//-------------------------------------------------------------
bool AdamDom::LoadXML(char *filename) {
  QFile f(filename);
  if (f.open(IO_ReadOnly)) {
    if (!layout->setContent(&f)) {
      printf("File Error: could not read xml file.\n");
      return false;
    }
  }
  else {
    printf("Could not find file '%s'.\n", filename);
    return false;
  }

  f.close();
  return true;
}


//-------------------------------------------------------------
// StartNode (private)
//   Returns: the first node of a DOM element
//-------------------------------------------------------------

QDomNode AdamDom::StartNode(QDomElement e) {
  return e.firstChild();
}


//-------------------------------------------------------------
// NextElement (private): looks for the next element among the
//      siblings of *n
//   n: pointer to a node from which to start the search
//   Returns: the first sibling of n which is an element
//-------------------------------------------------------------


QDomElement AdamDom::NextElement(QDomNode *n) {
  QDomElement e;

  while(!(*n).isNull()) {
    e = (*n).toElement();
    if (!e.isNull()) {
      (*n) = (*n).nextSibling();
      return e;
    }
    (*n) = (*n).nextSibling();
  }

  return n->toElement();
}


//--------------------------------------------------------------
// GetAttribute (public): read an attribute from current element
//  *attrib: name of the attribute
//  Returns: the value of "attrib"
//-------------------------------------------------------------

QString AdamDom::GetAttribute(const char *attrib) {
  if (currElem.hasAttribute(attrib))
    return currElem.attribute(attrib);
  else
    return "";
}


//--------------------------------------------------------------
// GetTagName (public)
//  Returns: the name of the current tag
//--------------------------------------------------------------

QString AdamDom::GetTagName() {
  return currElem.tagName();
}

//--------------------------------------------------------------
// RealCountEntries (private): counts the number of time
//      specified elements appear as a sibling or child of the 
//      given node.  The function is recursive, as it searches
//      through all the children of the node
//   Node: the node from which to start the search
//   entrylist: an array of the elements to search for.  The
//      elements are specified with a QString which is a list
//      describing the tag names and their parents, i.e.,
//      "PARENT1.PARENT2.PARENT3 . . . TAGNAME"
//   numentries: number of elements in entrylist
//   parents: keeps track of the parents of the node being
//      currently searched (remember the function is recursive).
//      A QString like entrylist.
//   Returns: number of elements found
//--------------------------------------------------------------


int AdamDom::RealCountEntries(QDomNode Node, QString *entrylist, int numentries,
		                          QString parents) {
  QDomElement Elem;
  int i, counter;

  counter = 0;

  Elem = NextElement(&Node);
  while (!Elem.isNull()) {
    for (i = 0; i < numentries; i++) {
      if (parents + "." + Elem.tagName() == entrylist[i])
        counter++;
    }
    counter += RealCountEntries(StartNode(Elem), entrylist, numentries,
				                        parents + "." + Elem.tagName());
    Elem = NextElement(&Node);
  }

  return counter;
}


//-------------------------------------------------------------
// RealGotoEntry (private): finds entry number "num" among the
//      siblings/children of the given node.  Works recursively
//      as it polls all the children.  Sets member variable
//      currElem to point to this entry.
//   Node: the node to begin the search at
//   parents: a list ".ELEMENT1.ELEMENT2. etc." describing the
//            parents of the current element
//   entrylist: an array of the elements to search for.  The
//      elements are specified with a QString which is a list
//      describing the tag names and their parents, i.e.,
//      "PARENT1.PARENT2.PARENT3 . . . TAGNAME"
//   numentries: number of elements in entrylist
//   parents: keeps track of the parents of the node being
//      currently searched (remember the function is recursive).
//      A QString like entrylist.
//   num: search for the "num"th occurence of anything in
//      entrylist
//   counter: current number of hits
//   Returns: current value of counter
//-------------------------------------------------------------


int AdamDom::RealGotoEntry(QDomNode Node, QString *entrylist, int numentries,
		                       QString parents, int num, int counter) {
  QDomElement Elem;
  int i;

  Elem = NextElement(&Node);
  while (!Elem.isNull()) {
    for (i = 0; i < numentries; i++) {
      if (parents + "." + Elem.tagName() == entrylist[i])
        counter++;
    }
    if (counter == num) {
      currElem = Elem;
      break;
    }
    counter = RealGotoEntry(StartNode(Elem), entrylist, numentries,
				parents + "." + Elem.tagName(), num, counter);
    if (counter >= num)
      break;
    Elem = NextElement(&Node);
  }

  return counter;
}


//-------------------------------------------------------------
// RealGotoTag (private): recursive private function that 
//      finds entry with tag name "entrylist" and attribute
//      "attrib" with value "val". Sets currElem to point to
//      this entry
//   Node: the node to begin the search at
//   parents: a list ".ELEMENT1.ELEMENT2. etc." describing the
//            parents of the current element
//   entrylist: an array of the elements to search for.  The
//      elements are specified with a QString which is a list
//      describing the tag names and their parents, i.e.,
//      "PARENT1.PARENT2.PARENT3 . . . TAGNAME"
//   *attrib: name of attribute to look for
//   val: value of attribute that we want
//   parents: keeps track of the parents of the node being
//      currently searched (remember the function is recursive).
//      A QString like entrylist.
//   Returns: true if it is found, false if not
//-------------------------------------------------------------


bool AdamDom::RealGotoTag(QDomNode Node, QString entrylist, char *attrib,
		                      QString val, QString parents) {
  QDomElement Elem;
  int i;

  Elem = NextElement(&Node);
  while (!Elem.isNull()) {
    if (parents + "." + Elem.tagName() == entrylist) {
      currElem = Elem;
      if (GetAttribute(attrib) == val) {
        return true;
      }
    }
    if (RealGotoTag(StartNode(Elem), entrylist, attrib, val, parents + "." +
					          Elem.tagName()))
      return true;
    Elem = NextElement(&Node);
  }

  return false;
}


//-------------------------------------------------------------
// CountEntries (public): public wrapper for RealCountEntries
//   StartChild: if true, starts looking from currElem,
//      otherwise, searches whole tree
//   See RealCountEntries for info on entrylist etc.
//   Returns: number of entries found
//-------------------------------------------------------------


int AdamDom::CountEntries(QString entrylist, bool StartChild) {
  QString templist[1];

  templist[0] = entrylist;
	if (!StartChild)
    return RealCountEntries(StartNode(layout->documentElement()),
				                    templist, 1, "");
	else
		return RealCountEntries(StartNode(currElem), templist, 1, "");
}

int AdamDom::CountEntries(QString *entrylist, int numentries,
		                      bool StartChild) {
	if (!StartChild)
    return RealCountEntries(StartNode(layout->documentElement()), entrylist,
				                    numentries, "");
	else
		return RealCountEntries(StartNode(currElem), entrylist, numentries, "");
}


//-------------------------------------------------------------
// GotoEntry (public): public wrapper for RealGotoEntry.
//      currElem is set to point to this entry
//   StartChild: if true, starts looking from currElem,
//      otherwise, searches whole tree
//   See RealGotoEntry for info on entrylist
//   num: search for the "num"th occurence of entrylist
//   Returns: true if found, false if not found
//-------------------------------------------------------------


bool AdamDom::GotoEntry(QString entrylist, int num, bool StartChild) {
  QString templist[1];

  templist[0] = entrylist;
  if (!StartChild) {
  	if (RealGotoEntry(StartNode(layout->documentElement()), templist, 1, "",
	  			            num + 1, 0) > num + 1)
      return false;
    else
      return true;
	}
	else {
  	if (RealGotoEntry(StartNode(currElem), templist, 1, "",
	  			            num + 1, 0) > num + 1)
      return false;
    else
      return true;
	}
}

bool AdamDom::GotoEntry(QString *entrylist, int numentries, int num,
		                    bool StartChild) {
  if (!StartChild) {
  	if (RealGotoEntry(StartNode(layout->documentElement()), entrylist,
					            numentries, "",
	  			                      num + 1, 0) > num + 1)
      return false;
    else
      return true;
	}
	else {
  	if (RealGotoEntry(StartNode(currElem), entrylist, numentries, "",
	  			            num + 1, 0) > num + 1)
      return false;
    else
      return true;
	}
}

//-------------------------------------------------------------
// GotoEntry (public): public wrapper for RealGotoEntry.
//      currElem is set to point to this entry
//   StartChild: if true, starts looking from currElem,
//      otherwise, searches whole tree
//   See RealGotoEntry for info on entrylist etc.
//   Returns: true if found, false if not found
//-------------------------------------------------------------


bool AdamDom::GotoTag(QString entrylist, char *attrib, char *val,
		                  bool StartChild) {
  if (!StartChild) {
  	if (RealGotoTag(StartNode(layout->documentElement()), entrylist,
					          attrib, val, ""))
      return true;
    else
      return false;
	}
	else {
  	if (RealGotoTag(StartNode(currElem), entrylist, attrib, val, ""))
      return true;
    else
      return false;
	}
}


//-------------------------------------------------------------
// AddBookMarks: make more bookmark slots
//   num: number to add
//-------------------------------------------------------------

void AdamDom::AddBookMarks(int num) {
	int i;

	for (i = 0; i < num; i++)
		BookMarks.append(new QDomElement);
}


//-------------------------------------------------------------
// SetBookMark: bookmark currElem
//   index: BookMark number
//-------------------------------------------------------------

void AdamDom::SetBookMark(int index) {
  QDomElement *e;
	
	if (index <= BookMarks.count()) {
		e = BookMarks.at(index);
		*e = currElem;
	}
}


//-------------------------------------------------------------
// GoBookMark: set currElem to specified bookmark
//   index: BookMark number (1 - 19)
//-------------------------------------------------------------

void AdamDom::GoBookMark(int index) {
  QDomElement *e;
	
	if (index <= BookMarks.count()) {
		e = BookMarks.at(index);
		currElem = *e;
	}
}


//-------------------------------------------------------------
// NextSib: go currElement's next sibling
//-------------------------------------------------------------

void AdamDom::GotoNextSib() {
  QDomNode n;

	n = currElem.nextSibling();
	currElem = NextElement(&n);	
}

//-------------------------------------------------------------
// FirstChild: go currElement's first child
//-------------------------------------------------------------

void AdamDom::GotoFirstChild() {
  QDomNode n;

	n = StartNode(currElem);
	currElem = NextElement(&n);	
}

//-------------------------------------------------------------
// NullEntry: checks currElem to see if it is NULL
//   Returns: true if NULL, false if not
//-------------------------------------------------------------

bool AdamDom::NullEntry() {
	if (currElem.isNull())
		return true;
	else
		return false;
}
