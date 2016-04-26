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

/* dom.c - document object model interface
modifications:
June.15.2005 - Sam Skuce (ATI Industrial Automation) and Gabriel Baud-Bovy - moved definition
of DOM_Exception into dom.c from dom.h.
 */

#include <limits.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "dom.h"

unsigned short DOM_Exception; // GBB: used to be in dom.h (now defined as extern)

/* prototypes required for node.c
 */

DOM_Node *Document_createNode(DOM_Document *doc, unsigned short nodeType);
NodeEntry *NodeList_remove(DOM_NodeList *nl, DOM_Node *oldChild);
NodeEntry *NodeList_append(DOM_NodeList *nl, DOM_Node *newChild);
DOM_NodeList *Document_createNodeList(DOM_Document *doc);
DOM_NamedNodeMap *Document_createNamedNodeMap(DOM_Document *doc);

/* DOM_Exception
 */

//DBL unsigned short DOM_Exception;

DOM_String *
DOM_Exception_message(unsigned short exception)
{
	switch (exception) {
		case DOM_NO_ERR:
			return "The operation was successfull.";
		case DOM_INDEX_SIZE_ERR:
			return "The index specified was out of range.";
		case DOM_HIERARCHY_REQUEST_ERR:
			return "The request violated tree hierarchy constraints.";
		case DOM_WRONG_DOCUMENT_ERR:
			return "The document context is invalid.";
		case DOM_INVALID_CHARACTER_ERR:
			return "An inappropriate character was encountered.";
		case DOM_NOT_FOUND_ERR:
			return "The specified node was not found.";
		case DOM_NOT_SUPPORTED_ERR:
			return "The requested operation is not supported.";
		case DOM_INUSE_ATTRIBUTE_ERR:
			return "The attribute is being used elsewhere.";
		case DOM_NO_MEMORY_ERR:
			return "No memory is available.";
		case DOM_NULL_POINTER_ERR:
			return "A parameter was null.";
		case DOM_SYSTEM_ERR:
			return "An external function error occured.";
		case DOM_XML_PARSER_ERR:
			return "An XML parser error occured.";
		case DOM_DOMSTRING_SIZE_ERR:
		case DOM_NO_DATA_ALLOWED_ERR:
		case DOM_NO_MODIFICATION_ALLOWED_ERR:
			break;
	}
	return "No description available.";
}


/* DOM_Implementation and DOM_ImplementationLS
 */

int
DOM_Implementation_hasFeature(DOM_String *feature, DOM_String *version)
{
	if (DOM_String_cmp(feature, "XML") == 0 &&
			(version == NULL || version[0] == '\0' || DOM_String_cmp(version, "1.0"))) {
		return 1;
	}
	return 0;
}
DOM_DocumentType *
DOM_Implementation_createDocumentType(DOM_String *qualifiedName,
						DOM_String *publicId, DOM_String systemId)
{
	return NULL;
}
DOM_Document *
DOM_Implementation_createDocument(DOM_String *namespaceURI,
						DOM_String *qualifiedName, DOM_DocumentType *doctype)
{
    DOM_Document *doc;

    doc = Document_createNode(NULL, DOM_DOCUMENT_NODE);
    if (doc) {
        doc->nodeName = "#document";
    }

    return doc;
}

DOM_String *
DOM_Element_getAttribute(const DOM_Element *element, const DOM_String *name)
{
	DOM_Node *node;
	DOM_String *r = NULL;

	if (element && name && element->attributes) {
		if ((node = DOM_NamedNodeMap_getNamedItem(element->attributes, name))) {
			r = DOM_String_dup(node->nodeValue);
		} else {
			r = DOM_String_dup("");
		}
		if (r == NULL) {
			DOM_Exception = DOM_NO_MEMORY_ERR;
			return NULL;
		}
	}

	return r;
}
void
DOM_Element_setAttribute(DOM_Element *element,
										const DOM_String *name, const DOM_String *value)
{
	if (element && name && value && element->attributes) {
		DOM_Attr *attr;

		attr = DOM_NamedNodeMap_getNamedItem(element->attributes, name);
		if (attr) {
			free(attr->nodeValue);
			attr->nodeValue = attr->u.Attr.value = DOM_String_dup(value);
			if (attr->nodeValue) {
			} else {
				DOM_Exception = DOM_NO_MEMORY_ERR;
				DOM_Document_destroyNode(attr->ownerDocument, attr);
			}
		} else {
			attr = DOM_Document_createAttribute(element->ownerDocument, name);
			if (attr) {
				free(attr->nodeValue);
				attr->nodeValue = attr->u.Attr.value = DOM_String_dup(value);
				if (attr->nodeValue) {
					DOM_NamedNodeMap_setNamedItem(element->attributes, attr);
				} else {
					DOM_Exception = DOM_NO_MEMORY_ERR;
					DOM_Document_destroyNode(attr->ownerDocument, attr);
				}
			}
		}
	}
}
void
DOM_Element_removeAttribute(DOM_Element *element, const DOM_String *name)
{
	if (element && name) {
		DOM_Attr *attr = DOM_NamedNodeMap_removeNamedItem(element->attributes, name);
		if (attr) {
			DOM_Document_destroyNode(attr->ownerDocument, attr);
		}
	}
}
DOM_Attr *
DOM_Element_getAttributeNode(const DOM_Element *element, const DOM_String *name)
{
	if (element && name) {
		return DOM_NamedNodeMap_getNamedItem(element->attributes, name);
	}
	return NULL;
}
DOM_Attr *
DOM_Element_setAttributeNode(DOM_Element *element, DOM_Attr *newAttr)
{
	if (element && newAttr) {
		return DOM_NamedNodeMap_setNamedItem(element->attributes, newAttr);
	}
	return NULL;
}
DOM_Attr *
DOM_Element_removeAttributeNode(DOM_Element *element, DOM_Attr *oldAttr)
{
	if (element && oldAttr && NodeList_remove(element->attributes, oldAttr)) {
		return oldAttr;
	}
	return NULL;
}

void
preorderTraversal(DOM_NodeList *list, DOM_Node *node, const DOM_String *tagname)
{
	DOM_Node *n;

	if (list && node && tagname) {
		if (DOM_String_cmp(tagname, node->nodeName) == 0) {
			NodeList_append(list, node);
		}
		for (n = node->firstChild; n != NULL; n = n->nextSibling) {
			preorderTraversal(list, n, tagname);
		}
	}
}

DOM_NodeList *
DOM_Element_getElementsByTagName(DOM_Element *element, const DOM_String *name)
{
	DOM_NodeList *list;

	if (element && name && (list = Document_createNodeList(element->ownerDocument))) {
		preorderTraversal(list, element, name);
		return list;
	}

    return NULL;
}
void
DOM_Element_normalize(DOM_Element *element)
{
	DOM_Node *node;
	DOM_Text *last = NULL;

	if (element) {
		for (node = element->firstChild; node != NULL; node = node->nextSibling) {
			if (node->nodeType == DOM_TEXT_NODE) {
				if (last) {
					DOM_CharacterData_insertData(node, 0, last->nodeValue);
					DOM_Node_removeChild(element, last);
					DOM_Document_destroyNode(last->ownerDocument, last);
					if (DOM_Exception) {
						return;
					}
				}
				last = node;
			} else {
				last = NULL;
				DOM_Element_normalize(node);
			}
		}
	}
}

DOM_String *
DOM_CharacterData_substringData(DOM_CharacterData *data,
                                            unsigned long offset, unsigned long count)
{
    DOM_String *sub;
	unsigned long dlen;

	if (data == NULL) {
		DOM_Exception = DOM_NULL_POINTER_ERR;
		return NULL;
	}
	if ((dlen = offset) >= data->u.CharacterData.length) {
		DOM_Exception = DOM_INDEX_SIZE_ERR;
		return NULL;
	}
	if (count > dlen || (offset + count) > dlen) {
		count = dlen - offset;
	}
	sub = malloc(count + 1);
	if (sub) {
		memcpy(sub, data->nodeValue + offset, count);
		sub[count] = '\0';
	} else {
		DOM_Exception = DOM_NO_MEMORY_ERR;
	}
	return sub;
}
void
DOM_CharacterData_appendData(DOM_CharacterData *data, const DOM_String *arg)
{
	DOM_String *str;
	DOM_String *tmp;
	unsigned long dlen, alen, tot;

	if (data == NULL) {
		DOM_Exception = DOM_NULL_POINTER_ERR;
		return;
	}
	if (arg) {
		dlen = data->u.CharacterData.length;
		alen = (unsigned long) DOM_String_len(arg);
		tot = dlen + alen;
		if ((str = malloc(tot + 1)) == NULL) {
			DOM_Exception = DOM_NO_MEMORY_ERR;
			return;
		}
		memcpy(str, data->nodeValue, dlen);
		memcpy(str + dlen, arg, alen);
		str[tot] = '\0';
		tmp = data->nodeValue;
		data->nodeValue = data->u.CharacterData.data = str;
		data->u.CharacterData.length = tot;
		free(tmp);
	}
}
void
DOM_CharacterData_insertData(DOM_CharacterData *data,
                                                unsigned long offset, const DOM_String *arg)
{
	DOM_String *str;
	DOM_String *tmp;
	unsigned long dlen, alen, tot;

	if (data == NULL) {
		DOM_Exception = DOM_NULL_POINTER_ERR;
		return;
	}
	if (arg) {
		if (offset >= (dlen = data->u.CharacterData.length)) {
			DOM_Exception = DOM_INDEX_SIZE_ERR;
			return;
		}
		alen = (unsigned long) DOM_String_len(arg);
		tot = dlen + alen;
		if ((str = malloc(tot + 1)) == NULL) {
			DOM_Exception = DOM_NO_MEMORY_ERR;
			return;
		}
		memcpy(str, data->nodeValue, offset);
		memcpy(str + offset, arg, alen);
		memcpy(str + offset + alen, data->nodeValue + offset, dlen - offset);
		str[tot] = '\0';
		tmp = data->nodeValue;
		data->nodeValue = data->u.CharacterData.data = str;
		data->u.CharacterData.length = tot;
		free(tmp);
	}
}
void
DOM_CharacterData_deleteData(DOM_CharacterData *data,
                                            unsigned long offset, unsigned long count)
{
	DOM_String *str;
	DOM_String *tmp;
	unsigned long dlen, tot, left;

	if (data == NULL) {
		DOM_Exception = DOM_NULL_POINTER_ERR;
		return;
	}
	if (offset < (dlen = data->u.CharacterData.length)) {
		DOM_Exception = DOM_INDEX_SIZE_ERR;
		return;
	}
	if (count > dlen || (offset + count) > dlen) {
		count = dlen - offset;
	}
	tot = dlen - count;
	left = dlen - offset - count;
	if ((str = malloc(tot + 1)) == NULL) {
		DOM_Exception = DOM_NO_MEMORY_ERR;
		return;
	}
	memcpy(str, data->nodeValue, offset);
	memcpy(str + offset, data->nodeValue + offset + count, left);
	str[tot] = '\0';
	tmp = data->nodeValue;
	data->nodeValue = data->u.CharacterData.data = str;
	data->u.CharacterData.length = tot;
	free(tmp);
}
void
DOM_CharacterData_replaceData(DOM_CharacterData *data, unsigned long offset,
                                                 unsigned long count, const DOM_String *arg)
{
	DOM_CharacterData_deleteData(data, offset, count);
	DOM_CharacterData_insertData(data, offset, arg);
}

DOM_Text *
DOM_Text_splitText(DOM_Text *text, unsigned long offset)
{
	DOM_Text *node;

	if (text && text->parentNode) {
		node = DOM_Document_createTextNode(text->ownerDocument, text->nodeValue + offset);
		if (node) {
			DOM_CharacterData_deleteData(text, offset, LONG_MAX);
			DOM_Node_insertBefore(text->parentNode, node, text->nextSibling);
			return node;
		}
	}

    return NULL;
}


DOM_Element *
DOM_Document_createElement(DOM_Document *doc, const DOM_String *tagName)
{
    DOM_Element *element;

    element = Document_createNode(doc, DOM_ELEMENT_NODE);
    if (element) {
        element->nodeName = element->u.Element.tagName = DOM_String_dup(tagName);
		element->attributes = Document_createNamedNodeMap(doc);
		if (element->nodeName == NULL || element->attributes == NULL) {
			DOM_Exception = DOM_NO_MEMORY_ERR;
			DOM_Document_destroyNode(doc, element);
			return NULL;
		}
    }

    return element;
}
DOM_DocumentFragment *
DOM_Document_createDocumentFragment(DOM_Document *doc)
{
    DOM_DocumentFragment *frag;

    frag = Document_createNode(doc, DOM_DOCUMENT_FRAGMENT_NODE);
    if (frag) {
        frag->nodeName = "#document-fragment";
    }

    return frag;
}
DOM_Text *
DOM_Document_createTextNode(DOM_Document *doc, const DOM_String *data)
{
    DOM_Text *text;

    text = Document_createNode(doc, DOM_TEXT_NODE);
    if (text) {
        text->nodeName = "#text";
        text->nodeValue = text->u.CharacterData.data = DOM_String_dup(data);
        if (text->nodeValue == NULL) {
			DOM_Exception = DOM_NO_MEMORY_ERR;
            DOM_Document_destroyNode(doc, text);
            return NULL;
        }
        text->u.CharacterData.length = (unsigned long) DOM_String_len(data);
    }

    return text;
}
DOM_Comment *
DOM_Document_createComment(DOM_Document *doc, const DOM_String *data)
{
    DOM_Comment *comment;

    comment = Document_createNode(doc, DOM_COMMENT_NODE);
    if (comment) {
        comment->nodeName = "#comment";
        comment->nodeValue = comment->u.CharacterData.data = DOM_String_dup(data);
        if (comment->nodeValue == NULL) {
			DOM_Exception = DOM_NO_MEMORY_ERR;
            DOM_Document_destroyNode(doc, comment);
            return NULL;
        }
        comment->u.CharacterData.length = (unsigned long) DOM_String_len(data);
    }

    return comment;
}
DOM_CDATASection *
DOM_Document_createCDATASection(DOM_Document *doc, const DOM_String *data)
{
    DOM_CDATASection *cdata;

    cdata = Document_createNode(doc, DOM_CDATA_SECTION_NODE);
    if (cdata) {
        cdata->nodeName = "#cdata-section";
        cdata->nodeValue = cdata->u.CharacterData.data = DOM_String_dup(data);
        if (cdata->u.CharacterData.data == NULL) {
			DOM_Exception = DOM_NO_MEMORY_ERR;
            DOM_Document_destroyNode(doc, cdata);
            return NULL;
        }
        cdata->u.CharacterData.length = (unsigned long) DOM_String_len(data);
    }

    return cdata;
}
DOM_ProcessingInstruction *
DOM_Document_createProcessingInstruction(DOM_Document *doc,
                                         const DOM_String *target, const DOM_String *data)
{
    DOM_ProcessingInstruction *pi;

    pi = Document_createNode(doc, DOM_PROCESSING_INSTRUCTION_NODE);
    if (pi) {
        pi->nodeName = pi->u.ProcessingInstruction.target = DOM_String_dup(target);
        pi->nodeValue = pi->u.ProcessingInstruction.data = DOM_String_dup(data);
        if (pi->nodeName == NULL || pi->nodeValue == NULL) {
			DOM_Exception = DOM_NO_MEMORY_ERR;
            DOM_Document_destroyNode(doc, pi);
            return NULL;
        }
    }

    return pi;
}
DOM_Attr *
DOM_Document_createAttribute(DOM_Document *doc, const DOM_String *name)
{
    DOM_Attr *attr;

    attr = Document_createNode(doc, DOM_ATTRIBUTE_NODE);
    if (attr) {
        attr->nodeName = attr->u.Attr.name = DOM_String_dup(name);
        attr->nodeValue = attr->u.Attr.value = DOM_String_dup("");
        if (attr->nodeName == NULL || attr->nodeValue == NULL) {
			DOM_Exception = DOM_NO_MEMORY_ERR;
            DOM_Document_destroyNode(doc, attr);
            return NULL;
        }
    }

    return attr;
}
DOM_EntityReference *
DOM_Docuement_createEntityReference(DOM_Document *doc, const DOM_String *name)
{
    DOM_EntityReference *eref;

    eref = Document_createNode(doc, DOM_ENTITY_REFERENCE_NODE);

    return eref;
}

DOM_NodeList *
DOM_Document_getElementsByTagName(DOM_Document *doc, const DOM_String *tagname)
{
	if (doc && tagname) {
		return DOM_Element_getElementsByTagName(doc->u.Document.documentElement, tagname);
	}

    return NULL;
}

/* Temporary functions */

void
DOM_Node_printNode(DOM_Node *node)
{
    if (node == NULL) {
        printf("node was null\n");
        return;
    }

    printf("nodeName=%s,nodeValue=%s,", node->nodeName, node->nodeValue);
    switch (node->nodeType) {
        case DOM_ELEMENT_NODE:
            printf("type=DOM_ELEMENT_NODE");
            break;
        case DOM_ATTRIBUTE_NODE:
            printf("type=DOM_ATTRIBUTE_NODE");
            break;
        case DOM_TEXT_NODE:
            printf("type=DOM_TEXT_NODE");
            break;
        case DOM_CDATA_SECTION_NODE:
            printf("type=DOM_CDATA_SECTION_NODE");
            break;
        case DOM_ENTITY_REFERENCE_NODE:
            printf("type=DOM_ENTITY_REFERENCE_NODE");
            break;
        case DOM_ENTITY_NODE:
            printf("type=DOM_ENTITY_NODE");
            break;
        case DOM_PROCESSING_INSTRUCTION_NODE:
            printf("type=DOM_PROCESSING_INSTRUCTION_NODE");
            break;
        case DOM_COMMENT_NODE:
            printf("type=DOM_COMMENT_NODE");
            break;
        case DOM_DOCUMENT_NODE:
            printf("type=DOM_DOCUMENT_NODE");
            break;
        case DOM_DOCUMENT_TYPE_NODE:
            printf("type=DOM_DOCUMENT_TYPE_NODE");
            break;
        case DOM_DOCUMENT_FRAGMENT_NODE:
            printf("type=DOM_DOCUMENT_FRAGMENT_NODE");
            break;
        case DOM_NOTATION_NODE:
            printf("type=DOM_NOTATION_NODE");
            break;
    }
    printf(",parentNode->nodeName=%s,firstChild->nodeName=%s", (node->parentNode == NULL ? "(null)" : node->parentNode->nodeName), (node->firstChild == NULL ? "(null)" : node->firstChild->nodeName));
	printf(",lastChild->nodeName=%s,childNodes->length=%lu", (node->lastChild == NULL ? "(null)" : node->lastChild->nodeName), (node->childNodes == NULL ? 0 : node->childNodes->length));
    printf(",previousSibling->nodeName=%s,nextSibling->nodeName=%s,attributes->length=%lu\n", (node->previousSibling == NULL ? "(null)" : node->previousSibling->nodeName), (node->nextSibling == NULL ? "(null)": node->nextSibling->nodeName), (node->attributes == NULL ? 0 : node->attributes->length));
    fflush(stdout);
}

//ATI_strdup added by DBL
char *ATI_strdup(const char *s) {
// Returns a pointer to a copy of the string s
	size_t t;
	char *p;
	t=strlen(s)+1;	// add one for termination character
	p = malloc(sizeof(char) * t);
	if (p==NULL) return p;
	*p='\0';
	strcat(p,s);
	return p;
}
