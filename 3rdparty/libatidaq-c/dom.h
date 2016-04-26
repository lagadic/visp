/* domc document object model library in c
 * Copyright (c) 2001 Michael B. Allen <mballen@erols.com>
 *
 * The MIT License
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

/* dom.h - document object model interface
modifications:
june.15.2005 - Sam Skuce (ATI Industrial Automation) and Gabriel Baud-Bovy - 
changed DOM_Exception to an extern to avoid redefinition problems
 */

#ifndef DOM_H
#define DOM_H

#include <stdlib.h>
#include <string.h>
#include <time.h>

/* DOM_String
 */

typedef char DOM_String;

#define DOM_String_dup ATI_strdup	// changed from strdup to ATI_strdup by DBL
char *ATI_strdup(const char *s);	// line added by DBL

#define DOM_String_len strlen
#define DOM_String_cmp strcmp

/* DOM_TimeStamp - Introduced in DOM Level 2
 */

typedef time_t DOM_TimeStamp;

/* DOM_Exception
 */

extern unsigned short DOM_Exception; // GBB: added extern (declaration moved in dom.c)

#define DOM_NO_ERR                      0
#define DOM_INDEX_SIZE_ERR              1
#define DOM_DOMSTRING_SIZE_ERR          2
#define DOM_HIERARCHY_REQUEST_ERR       3
#define DOM_WRONG_DOCUMENT_ERR          4
#define DOM_INVALID_CHARACTER_ERR       5
#define DOM_NO_DATA_ALLOWED_ERR         6
#define DOM_NO_MODIFICATION_ALLOWED_ERR 7
#define DOM_NOT_FOUND_ERR               8
#define DOM_NOT_SUPPORTED_ERR           9
#define DOM_INUSE_ATTRIBUTE_ERR         10
#define DOM_NO_MEMORY_ERR               11
#define DOM_NULL_POINTER_ERR            12
#define DOM_SYSTEM_ERR                  13
#define DOM_XML_PARSER_ERR              14

DOM_String *DOM_Exception_message(unsigned short);

/* DOM_EventException - Introduced in DOM Level 2
 */

extern unsigned short DOM_EventException;

#define DOM_UNSPECIFIED_EVENT_TYPE_ERR  0



/* DOM_Node
 */

typedef struct DOM_Node DOM_Node;
typedef struct DOM_NodeList DOM_NodeList;
typedef struct DOM_NodeList DOM_NamedNodeMap;
typedef struct NodeEntry NodeEntry;

typedef DOM_Node DOM_Attr;
typedef DOM_Node DOM_Element;
typedef DOM_Node DOM_CharacterData;
typedef DOM_CharacterData DOM_Text;
typedef DOM_CharacterData DOM_Comment;
typedef DOM_Text DOM_CDATASection;
typedef DOM_Node DOM_DocumentType;
typedef DOM_Node DOM_Notation;
typedef DOM_Node DOM_Entity;
typedef DOM_Node DOM_EntityReference;
typedef DOM_Node DOM_ProcessingInstruction;
typedef DOM_Node DOM_DocumentFragment;
typedef DOM_Node DOM_Document;
typedef DOM_Document DOM_DocumentLS;
/* Introduced in DOM Level 2: */
typedef DOM_Node DOM_EventTarget;
typedef struct ListenerEntry ListenerEntry;
typedef DOM_Document DOM_DocumentEvent;
typedef DOM_Document DOM_AbstractView;
typedef DOM_Document DOM_DocumentView;

#define DOM_ELEMENT_NODE                1
#define DOM_ATTRIBUTE_NODE              2
#define DOM_TEXT_NODE                   3
#define DOM_CDATA_SECTION_NODE          4
#define DOM_ENTITY_REFERENCE_NODE       5
#define DOM_ENTITY_NODE                 6
#define DOM_PROCESSING_INSTRUCTION_NODE 7
#define DOM_COMMENT_NODE                8
#define DOM_DOCUMENT_NODE               9
#define DOM_DOCUMENT_TYPE_NODE          10
#define DOM_DOCUMENT_FRAGMENT_NODE      11
#define DOM_NOTATION_NODE               12

/* events forward references - Introduced in DOM Level 2
 */

typedef struct DOM_Event DOM_Event;
typedef struct DOM_Event DOM_UIEvent;
typedef DOM_UIEvent DOM_KeyEvent;
typedef struct DOM_Event DOM_MutationEvent;

typedef void (*DOM_EventListener)(DOM_Event *evt);

struct ListenerEntry {
	DOM_String *type;
	DOM_EventListener listener_fn;
	int useCapture;
};

struct DOM_Node {
    DOM_String *nodeName;
    DOM_String *nodeValue;
    unsigned short nodeType;
    DOM_Node *parentNode;
    DOM_NodeList *childNodes;
    DOM_Node *firstChild;
    DOM_Node *lastChild;
    DOM_Node *previousSibling;
    DOM_Node *nextSibling;
    DOM_NamedNodeMap *attributes;
    DOM_Document *ownerDocument;
	/* Custom Fields */
	void *userPtr;
	unsigned int listeners_len;
	ListenerEntry **listeners;
    union {
        struct {
            DOM_String *name;
            int specified;
            DOM_String *value;
        } Attr;
        struct {
            DOM_String *tagName;
        } Element;
        struct {
            DOM_String *name;
            DOM_NamedNodeMap *entities;
            DOM_NamedNodeMap *notations;
        } DocumentType;
        struct {
            DOM_String *data;
            unsigned long length;
        } CharacterData;
        struct {
            DOM_String *publicId;
            DOM_String *systemId;
        } Notation;
        struct {
            DOM_String *publicId;
            DOM_String *systemId;
            DOM_String *notationName;
        } Entity;
        struct {
            DOM_String *target;
            DOM_String *data;
        } ProcessingInstruction;
        struct {
            DOM_DocumentType *doctype;
            DOM_Element *documentElement;
			/* DOM_Document implements both DOM_DocumentView and DOM_AbstractView? */
			DOM_DocumentView *document;
			DOM_AbstractView *defaultView;
        } Document;
    } u;
};

DOM_Node *DOM_Node_insertBefore(DOM_Node *node, DOM_Node *newChild, DOM_Node *refChild);
DOM_Node *DOM_Node_replaceChild(DOM_Node *node, DOM_Node *newChild, DOM_Node *oldChild);
DOM_Node *DOM_Node_removeChild(DOM_Node *node, DOM_Node *oldChild);
DOM_Node *DOM_Node_appendChild(DOM_Node *node, DOM_Node *newChild);
int DOM_Node_hasChildNodes(const DOM_Node *node);
DOM_Node *DOM_Node_cloneNode(DOM_Node *node, int deep);

/* DOM_NodeList, DOM_NamedNodeMap
 */

struct NodeEntry {
	NodeEntry *prev;
	NodeEntry *next;
	DOM_Node *node;
};
struct DOM_NodeList {
	unsigned long length;
	NodeEntry *first;
	NodeEntry *last;
};

DOM_Node *DOM_NodeList_item(const DOM_NodeList *nl, unsigned long index);

DOM_Node *DOM_NamedNodeMap_getNamedItem(const DOM_NamedNodeMap *map, const DOM_String *name);
DOM_Node *DOM_NamedNodeMap_setNamedItem(DOM_NamedNodeMap *map, DOM_Node *arg);
DOM_Node *DOM_NamedNodeMap_removeNamedItem(DOM_NamedNodeMap *map, const DOM_String *name);
DOM_Node *DOM_NamedNodeMap_item(const DOM_NamedNodeMap *map, unsigned long index);

/* DOM_Implementation
 */

int DOM_Implementation_hasFeature(DOM_String *feature, DOM_String *version);
DOM_DocumentType *DOM_Implementation_createDocumentType(DOM_String *qualifiedName,
										DOM_String *publicId, DOM_String systemId);
DOM_Document *DOM_Implementation_createDocument(DOM_String *namespaceURI,
										DOM_String *qualifiedName, DOM_DocumentType *doctype);

/* DOM_Element
 */

DOM_String *DOM_Element_getAttribute(const DOM_Element *element, const DOM_String *name);
void DOM_Element_setAttribute(DOM_Element *element, const DOM_String *name,
										const DOM_String *value);
void DOM_Element_removeAttribute(DOM_Element *element, const DOM_String *name);
DOM_Attr *DOM_Element_getAttributeNode(const DOM_Element *element, const DOM_String *name);
DOM_Attr *DOM_Element_setAttributeNode(DOM_Element *element, DOM_Attr *newAttr);
DOM_Attr *DOM_Element_removeAttributeNode(DOM_Element *element, DOM_Attr *oldAttr);
DOM_NodeList *DOM_Element_getElementsByTagName(DOM_Element *element, const DOM_String *name);
void DOM_Element_normalize(DOM_Element *element);

/* DOM_CharacterData
 */

DOM_String *DOM_CharacterData_substringData(DOM_CharacterData *data,
										unsigned long offset, unsigned long count);
void DOM_CharacterData_appendData(DOM_CharacterData *data, const DOM_String *arg);
void DOM_CharacterData_insertData(DOM_CharacterData *data, unsigned long offset,
										const DOM_String *arg);
void DOM_CharacterData_deleteData(DOM_CharacterData *data, unsigned long offset,
										unsigned long count);
void DOM_CharacterData_replaceData(DOM_CharacterData *data, unsigned long offset,
										unsigned long count, const DOM_String *arg);

/* DOM_Text
 */

DOM_Text *DOM_Text_splitText(DOM_Text *text, unsigned long offset);

/* DOM_Document
 */

DOM_Element *DOM_Document_createElement(DOM_Document *doc, const DOM_String *tagName);
DOM_DocumentFragment *DOM_Document_createDocumentFragment(DOM_Document *doc);
DOM_Text *DOM_Document_createTextNode(DOM_Document *doc, const DOM_String *data);
DOM_Comment *DOM_Document_createComment(DOM_Document *doc, const DOM_String *data);
DOM_CDATASection *DOM_Document_createCDATASection(DOM_Document *doc, const DOM_String *data);
DOM_ProcessingInstruction *DOM_Document_createProcessingInstruction(DOM_Document *doc,
										const DOM_String *target, const DOM_String *data);
DOM_Attr *DOM_Document_createAttribute(DOM_Document *doc, const DOM_String *name);
DOM_EntityReference *DOM_Docuement_createEntityReference(DOM_Document *doc,
										const DOM_String *name);
DOM_NodeList *DOM_Document_getElementsByTagName(DOM_Document *doc, const DOM_String *tagname);

void DOM_Document_destroyNode(DOM_Document *doc, DOM_Node *node);
void DOM_Document_destroyNodeList(DOM_Document *doc, DOM_NodeList *nl, int free_nodes);
void DOM_Document_destroyNamedNodeMap(DOM_Document *doc, DOM_NamedNodeMap *nnm, int free_nodes);

/* DOM_DocumentLS - Based roughly on Load and Save WD
 * http://www.w3.org/TR/2001/WD-DOM-Level-3-ASLS-20010607/load-save.html
 */

int DOM_DocumentLS_load(DOM_DocumentLS *doc, const DOM_String *uri);
int DOM_DocumentLS_save(DOM_DocumentLS *doc, const DOM_String *uri, const DOM_Node *node);

/* Events - Introduced in DOM Level 2
 * http://www.w3.org/TR/2000/REC-DOM-Level-2-Events-20001113/events.html
 */

/* DOM_Event - Introduced in DOM Level 2
 */

#define DOM_EVENT_CAPTURING_PHASE 1
#define DOM_EVENT_AT_TARGET       2
#define DOM_EVENT_BUBBLING_PHASE  3

struct DOM_Event {
	DOM_String *type;
	DOM_EventTarget *target;
	DOM_EventTarget *currentTarget;
	unsigned short eventPhase;
	int bubbles;
	int cancelable;
	DOM_TimeStamp timeStamp;
	/* custom -- do not touch */
	int pd;
	int sp;
	/* UIEvent members */
	DOM_AbstractView *view;
	long detail;
	union {
		struct {
			DOM_String *outputString;
			unsigned long keyVal;
			unsigned long virtKeyVal;
			int inputGenerated;
			int numPad;
		} KeyEvent;
//DBL		struct {
			/* maybe later */
//DBL		} MouseEvent;
		struct {
			DOM_Node *relatedNode;
			DOM_String *prevValue;
			DOM_String *newValue;
			DOM_String *attName;
			unsigned short attrChange;
		} MutationEvent;
	} u;
};

void DOM_Event_stopPropagation(DOM_Event *evt);
void DOM_Event_preventDefault(DOM_Event *evt);
void DOM_Event_initEvent(DOM_Event *evt, const DOM_String *eventTypeArg,
										int canBubbleArg, int cancelableArg);

/* DOM_DocumentEvent - Introduced in DOM Level 2
 */

DOM_Event *DOM_DocumentEvent_createEvent(DOM_DocumentEvent *doc,
										const DOM_String *eventType);
void DOM_DocumentEvent_destroyEvent(DOM_DocumentEvent *doc, DOM_Event *evt);

/* DOM_UIEvent
 */

void DOM_UIEvent_initUIEvent(DOM_UIEvent *evt,
										DOM_String *typeArg,
										int canBubbleArg,
										int cancelableArg,
										DOM_AbstractView *viewArg,
										long detailArg);

/* DOM_EventTarget - Introduced in DOM Level 2
 */

void DOM_EventTarget_addEventListener(DOM_EventTarget *target, const DOM_String *type,
										DOM_EventListener listener, int useCapture);
void DOM_EventTarget_removeEventListener(DOM_EventTarget *target, const DOM_String *type,
										DOM_EventListener listener, int useCapture);
int DOM_EventTarget_dispatchEvent(DOM_EventTarget *target, DOM_Event *evt);

/* DOM_KeyEvent - Introduced in DOM Level 3
 * http://www.w3.org/TR/2001/WD-DOM-Level-3-Events-20010410/events.html
 */

int DOM_KeyEvent_checkModifier(unsigned long modifier);
void DOM_KeyEvent_initKeyEvent(DOM_KeyEvent *evt, const DOM_String *typeArg,
										int canBubbleArg, int cancelableArg,
										DOM_AbstractView *viewArg, unsigned short detailArg,
										DOM_String *outputStringArg,
										unsigned long keyValArg,
										unsigned long virtKeyValArg,
										int inputGeneratedArg,
										int numPadArg);
void DOM_KeyEvent_initModifier(unsigned long modifier, int value);

/* MutationEvent
 */

#define DOM_MUTATION_EVENT_MODIFICATION 1
#define DOM_MUTATION_EVENT_ADDITION     2
#define DOM_MUTATION_EVENT_REMOVAL      3

void DOM_MutationEvent_initMutationEvent(DOM_MutationEvent *evt,
										DOM_String *typeArg,
										int canBubbleArg,
										int cancelableArg,
										DOM_Node *relatedNodeArg,
										DOM_String *prevValueArg,
										DOM_String *newValueArg,
										DOM_String *attrNameArg,
										unsigned short attrChangeArg);


/* Temporary Functions
 */

void DOM_Node_printNode(DOM_Node *node);
char *target2str(char *buf, DOM_EventTarget *target);

#endif /* DOM_H */

