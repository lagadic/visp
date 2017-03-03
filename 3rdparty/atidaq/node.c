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

/* node.c - DOM_NodeList, DOM_NamedNodeMap, and DOM_Node
 */

#include "dom.h"

/* NodeList
 */

DOM_NodeList *
Document_createNodeList(DOM_Document *doc)
{
	DOM_NodeList *r;

	if ((r = calloc(sizeof *r, 1)) == NULL) {
		DOM_Exception = DOM_NO_MEMORY_ERR;
	}

	return r;
}
void
DOM_Document_destroyNodeList(DOM_Document *doc, DOM_NodeList *nl, int free_nodes)
{
	if (nl) {
		NodeEntry *e, *tmp;

		e = nl->first;
		while (e != NULL) {
			if (free_nodes) {
				DOM_Document_destroyNode(doc, e->node);
			}
			tmp = e;
			e = e->next;
			free(tmp); 
		}
		free(nl);
	}
}

DOM_Node *
DOM_NodeList_item(const DOM_NodeList *list, unsigned long index)
{
    if (list && index < list->length) {
		NodeEntry *e;

        for (e = list->first; e != NULL; e = e->next, index--) {
			if (index == 0) {
				return e->node;
			}
        }
    }

    return NULL;
}
NodeEntry *
NodeList_insert(DOM_NodeList *nl, DOM_Node *newChild, DOM_Node *refChild)
{
	NodeEntry *e;

	if (nl == NULL) {
		DOM_Exception = DOM_NULL_POINTER_ERR;
		return NULL;
	}

	if ((e = calloc(sizeof *e, 1)) == NULL) {
		DOM_Exception = DOM_NO_MEMORY_ERR;
		return NULL;
	}

	e->node = newChild;
	if (nl->length == 0) {
		nl->first = nl->last = e;
	} else if (refChild == NULL) {
		e->prev = nl->last;
		nl->last->next = e;
		nl->last = e;
	} else {
		NodeEntry *s;

		for (s = nl->first; s != NULL && s->node != refChild; s = s->next) {
			;
		}
		if (s == NULL || s->node != refChild) {
			DOM_Exception = DOM_NOT_FOUND_ERR;
			free(e);
			return NULL;
		}

		e->prev = s->prev;
		e->next = s;
		if (s == nl->first) {
			nl->first = e;
		} else {
			s->prev->next = e;
		}
		s->prev = e;
	}
	nl->length++;

	return e;
}
NodeEntry *
NodeList_replace(DOM_NodeList *nl, DOM_Node *newChild, DOM_Node *oldChild)
{
	NodeEntry *e;

	if (nl == NULL) {
		DOM_Exception = DOM_NULL_POINTER_ERR;
		return NULL;
	}

	for (e = nl->first; e != NULL && e->node != oldChild; e = e->next) {
		;
	}
	if (e == NULL) {
		DOM_Exception = DOM_NOT_FOUND_ERR;
		return NULL;
	}
	e->node = newChild;

	return e;
}
NodeEntry *
NodeList_remove(DOM_NodeList *nl, DOM_Node *oldChild)
{
	NodeEntry *e;

	if (nl == NULL) {
		DOM_Exception = DOM_NULL_POINTER_ERR;
		return NULL;
	}

	for (e = nl->first; e != NULL && e->node != oldChild; e = e->next) {
		;
	}
	if (e == NULL) {
		return NULL;
	}

	if (nl->first == nl->last) {
		nl->first = nl->last = NULL;
	} else if (e == nl->first) {
		nl->first = e->next;
		nl->first->prev = NULL;
	} else if (e == nl->last) {
		nl->last = e->prev;
		nl->last->next = NULL;
	} else {
		e->prev->next = e->next;
		e->next->prev = e->prev;
	}
	nl->length--;

	return e;
}
NodeEntry *
NodeList_append(DOM_NodeList *nl, DOM_Node *newChild)
{
	NodeEntry *e;

	if (nl == NULL) {
		DOM_Exception = DOM_NULL_POINTER_ERR;
		return NULL;
	}
	if ((e = calloc(sizeof *e, 1)) == NULL) {
		DOM_Exception = DOM_NO_MEMORY_ERR;
		return NULL;
	}

	e->node = newChild;
	if (nl->first == NULL) {
		nl->first = nl->last = e;
	} else {
		nl->last->next = e;
		e->prev = nl->last;
		nl->last = e;
	}

	nl->length++;

	return e;
}

/* NamedNodeMap
 */

void
DOM_Document_destroyNamedNodeMap(DOM_Document *doc, DOM_NamedNodeMap *map, int free_nodes)
{
    DOM_Document_destroyNodeList(doc, map, free_nodes);
}
DOM_NamedNodeMap *
Document_createNamedNodeMap(DOM_Document *doc)
{
    return Document_createNodeList(doc);
}

DOM_Node *
DOM_NamedNodeMap_getNamedItem(const DOM_NamedNodeMap *map, const DOM_String *name)
{
	NodeEntry *e;

	if (map && name) {
		for (e = map->first; e != NULL; e = e->next) {
			if (DOM_String_cmp(name, e->node->nodeName) == 0) {
				return e->node;
			}
		}
	}

	return NULL;
}
DOM_Node *
DOM_NamedNodeMap_setNamedItem(DOM_NamedNodeMap *map, DOM_Node *arg)
{
	NodeEntry *e;

	if (map && arg) {
		for (e = map->first; e != NULL && arg != e->node; e = e->next) {
			;
		}
		if (e) {
			DOM_Node *tmp = e->node;
			e->node = arg;
			return tmp;
		}
		NodeList_append(map, arg);
	}

	return NULL;
}
DOM_Node *
DOM_NamedNodeMap_removeNamedItem(DOM_NamedNodeMap *map, const DOM_String *name)
{
	NodeEntry *e;
	DOM_Node *r = NULL;

	if (map && name) {
		for (e = map->first; e != NULL; e = e->next) {
			if (DOM_String_cmp(name, e->node->nodeName) == 0 &&
												NodeList_remove(map, e->node)) {
				r = e->node;
				free(e);
				break;
			}
		}
	}

	return r;
}
DOM_Node *
DOM_NamedNodeMap_item(const DOM_NamedNodeMap *map, unsigned long index)
{
	if (map && index < map->length) {
		NodeEntry *e;

		for (e = map->first; e != NULL; e = e->next, index--) {
			if (index == 0) {
				return e->node;
			}
		}
	}

	return NULL;
}

/* Node
 */

void
DOM_Document_destroyNode(DOM_Document *doc, DOM_Node *node)
{
	if (node == NULL) {
        return;
    }

    if (node->childNodes) {
        DOM_Document_destroyNodeList(doc, node->childNodes, 1);
    }
	if (node->listeners) {
		unsigned int i;

		for (i = 0; i < node->listeners_len; i++) {
			if (node->listeners[i]) {
				free(node->listeners[i]->type);
				free(node->listeners[i]);
			}
		}
		free(node->listeners);
	}

    switch(node->nodeType) {
        case DOM_ELEMENT_NODE:
			DOM_Document_destroyNamedNodeMap(doc, node->attributes, 1);
            free(node->nodeName);
            break;
        case DOM_TEXT_NODE:
        case DOM_COMMENT_NODE:
        case DOM_CDATA_SECTION_NODE:
            free(node->nodeValue);
            break;
        case DOM_ATTRIBUTE_NODE:
            free(node->nodeName);
            free(node->nodeValue);
            break;
        case DOM_ENTITY_REFERENCE_NODE:
        case DOM_ENTITY_NODE:
            break;
        case DOM_PROCESSING_INSTRUCTION_NODE:
            free(node->nodeName);
            free(node->nodeValue);
            break;
        case DOM_DOCUMENT_NODE:
        case DOM_DOCUMENT_TYPE_NODE:
        case DOM_NOTATION_NODE:
			break;
    }
    free(node);
}
DOM_Node *
Document_createNode(DOM_Document *doc, unsigned short nodeType)
{
    DOM_Node *node;

	if (nodeType != DOM_DOCUMENT_NODE && doc == NULL) {
		DOM_Exception = DOM_NULL_POINTER_ERR;
		return NULL;
	}

    node = calloc(sizeof *node, 1);
    if (node == NULL) {
		DOM_Exception = DOM_NO_MEMORY_ERR;
		return NULL;
	}

    node->nodeType = nodeType;
    node->ownerDocument = doc;
    switch (nodeType) {
        case DOM_DOCUMENT_NODE:
			node->ownerDocument = node;
        case DOM_ELEMENT_NODE:
        case DOM_ATTRIBUTE_NODE:
        case DOM_ENTITY_REFERENCE_NODE:
        case DOM_ENTITY_NODE:
        case DOM_DOCUMENT_FRAGMENT_NODE:
            node->childNodes = Document_createNodeList(doc);
            if (node->childNodes == NULL) {
                DOM_Document_destroyNode(doc, node);
                return NULL;
            }
    }

    return node;
}

#define CANNOT_HAVE_CHILDREN(t) ((t) == DOM_PROCESSING_INSTRUCTION_NODE || \
					(t) == DOM_COMMENT_NODE || (t) == DOM_TEXT_NODE || \
					(t) == DOM_CDATA_SECTION_NODE || (t) == DOM_NOTATION_NODE)

int
Node_isAncestor(const DOM_Node *node, const DOM_Node *parent)
{
	if (parent == NULL) {
		return 0;
	} else if (node == parent) {
		return 1;
	}
	return Node_isAncestor(node, parent->parentNode);
}

DOM_Node *
DOM_Node_insertBefore(DOM_Node *node, DOM_Node *newChild, DOM_Node *refChild)
{
	NodeEntry *e;
	int addingDocumentElement;

    if (node == NULL || newChild == NULL) {
		DOM_Exception = DOM_NULL_POINTER_ERR;
		return NULL;
	}
	if (newChild->ownerDocument != node->ownerDocument) {
		DOM_Exception = DOM_WRONG_DOCUMENT_ERR;
		return NULL;
	}
	addingDocumentElement = node->nodeType == DOM_DOCUMENT_NODE &&
							newChild->nodeType == DOM_ELEMENT_NODE;
	if ((refChild != NULL && refChild->parentNode != node) ||
							CANNOT_HAVE_CHILDREN(node->nodeType) ||
							Node_isAncestor(newChild, node) ||
							(addingDocumentElement && node->u.Document.documentElement)) {
		DOM_Exception = DOM_HIERARCHY_REQUEST_ERR;
		return NULL;
	}

	if (newChild->nodeType == DOM_DOCUMENT_FRAGMENT_NODE) {
		DOM_Node *n, *nxt;

		for (n = newChild->firstChild; n != NULL; n = nxt) {
			nxt = n->nextSibling;
			if (DOM_Node_removeChild(newChild, n) == NULL) {
				return NULL;
			}
			if (DOM_Node_insertBefore(node, n, refChild) == NULL) {
				DOM_Document_destroyNode(n->ownerDocument, n);
				return NULL;
			}
		}
		return newChild;
	}

	if ((e = NodeList_insert(node->childNodes, newChild, refChild)) == NULL) {
        return NULL;
    }

	if (node->firstChild == NULL) {
		node->firstChild = node->lastChild = newChild;
		newChild->previousSibling = NULL;
    	newChild->nextSibling = NULL;
	} else if (refChild == NULL) {
		newChild->previousSibling = node->lastChild;
		node->lastChild->nextSibling = newChild;
		node->lastChild = newChild;
    	newChild->nextSibling = NULL;
	} else {
		newChild->previousSibling = refChild->previousSibling;
		newChild->nextSibling = refChild;
		if (refChild == node->firstChild) {
			node->firstChild = newChild;
			newChild->previousSibling = NULL;
		} else {
			refChild->previousSibling->nextSibling = newChild;
		}
		refChild->previousSibling = newChild;
	}
    newChild->parentNode = node;

	if (addingDocumentElement) {
		node->u.Document.documentElement = newChild;
	}

    return newChild;
}
DOM_Node *
DOM_Node_replaceChild(DOM_Node *node, DOM_Node *newChild, DOM_Node *oldChild)
{
	DOM_Node *tmp;
	int addingDocumentElement;

    if (node == NULL || newChild == NULL || oldChild == NULL) {
		DOM_Exception = DOM_NULL_POINTER_ERR;
		return NULL;
	}
	if (newChild->ownerDocument != node->ownerDocument ||
							oldChild->ownerDocument != node->ownerDocument) {
		DOM_Exception = DOM_WRONG_DOCUMENT_ERR;
		return NULL;
	}
	addingDocumentElement = node->nodeType == DOM_DOCUMENT_NODE &&
							newChild->nodeType == DOM_ELEMENT_NODE;
	if (oldChild->parentNode != node || Node_isAncestor(newChild, node) ||
							(addingDocumentElement && node->u.Document.documentElement &&
							oldChild->nodeType != DOM_ELEMENT_NODE)) {
		DOM_Exception = DOM_HIERARCHY_REQUEST_ERR;
        return NULL;
    }

	for (tmp = node->firstChild; tmp != NULL && tmp != oldChild; tmp = tmp->nextSibling) {
		;
	}
	if (tmp != oldChild) {
		DOM_Exception = DOM_NOT_FOUND_ERR;
		return NULL;
	}

	if (newChild->nodeType == DOM_DOCUMENT_FRAGMENT_NODE) {
		DOM_Node *n, *nxt;

		for (n = newChild->firstChild; n != NULL; n = nxt) {
			nxt = n->nextSibling;
			if (DOM_Node_removeChild(newChild, n) == NULL) {
				return NULL;
			}
			if (DOM_Node_insertBefore(node, n, oldChild) == NULL) {
				DOM_Document_destroyNode(n->ownerDocument, n);
				return NULL;
			}
		}

		if (DOM_Node_removeChild(node, oldChild) == NULL) {
			return NULL;
		}

		return oldChild;
	}

	DOM_Node_removeChild(node, newChild);

	if (NodeList_replace(node->childNodes, newChild, oldChild) == NULL) {
		return NULL;
	}

	node->firstChild = node->childNodes->first->node;
	node->lastChild = node->childNodes->last->node;

    if ((newChild->previousSibling = oldChild->previousSibling)) {
        newChild->previousSibling->nextSibling = newChild;
    }
    if ((newChild->nextSibling = oldChild->nextSibling)) {
        newChild->nextSibling->previousSibling = newChild;
    }

    newChild->parentNode = node;
    oldChild->parentNode = NULL;
    oldChild->previousSibling = NULL;
    oldChild->nextSibling = NULL;

	if (addingDocumentElement) {
		node->u.Document.documentElement = newChild;
	}

    return oldChild;
}
DOM_Node *
DOM_Node_removeChild(DOM_Node *node, DOM_Node *oldChild)
{
	NodeEntry *e;

    if (node == NULL || oldChild == NULL) {
		DOM_Exception = DOM_NULL_POINTER_ERR;
		return NULL;
	}
	if (oldChild->ownerDocument != node->ownerDocument) {
		DOM_Exception = DOM_WRONG_DOCUMENT_ERR;
		return NULL;
	}

	if ((e = NodeList_remove(node->childNodes, oldChild)) == NULL) {
		return NULL;
	} else {
		free(e);
	}

	if (node->firstChild == node->lastChild) {
		node->firstChild = node->lastChild = NULL;
	} else if (oldChild == node->firstChild) {
		node->firstChild = oldChild->nextSibling;
		node->firstChild->previousSibling = NULL;
	} else if (oldChild == node->lastChild) {
		node->lastChild = oldChild->previousSibling;
		node->lastChild->nextSibling = NULL;
	} else {
        oldChild->previousSibling->nextSibling = oldChild->nextSibling;
        oldChild->nextSibling->previousSibling = oldChild->previousSibling;
	}

    oldChild->previousSibling = NULL;
    oldChild->nextSibling = NULL;
    oldChild->parentNode = NULL;

	if (node->nodeType == DOM_DOCUMENT_NODE && oldChild->nodeType == DOM_ELEMENT_NODE) {
		node->u.Document.documentElement = NULL;
	}

    return oldChild;
}
DOM_Node *
DOM_Node_appendChild(DOM_Node *node, DOM_Node *newChild)
{
	int addingDocumentElement;

    if (node == NULL || newChild == NULL) {
		DOM_Exception = DOM_NULL_POINTER_ERR;
        return NULL;
    }
	if (newChild->ownerDocument != node->ownerDocument) {
		DOM_Exception = DOM_WRONG_DOCUMENT_ERR;
		return NULL;
	}
	addingDocumentElement = node->nodeType == DOM_DOCUMENT_NODE &&
							newChild->nodeType == DOM_ELEMENT_NODE;
	if (CANNOT_HAVE_CHILDREN(node->nodeType) || Node_isAncestor(newChild, node) ||
							(addingDocumentElement && node->u.Document.documentElement)) {
		DOM_Exception = DOM_HIERARCHY_REQUEST_ERR;
		return NULL;
	}

	if (newChild->nodeType == DOM_DOCUMENT_FRAGMENT_NODE) {
		DOM_Node *n, *nxt;

		for (n = newChild->firstChild; n != NULL; n = nxt) {
			nxt = n->nextSibling;
			if (DOM_Node_removeChild(newChild, n) == NULL) {
				return NULL;
			}
			if (DOM_Node_appendChild(node, n) == NULL) {
				DOM_Document_destroyNode(n->ownerDocument, n);
				return NULL;
			}
		}
		return newChild;
	}

	DOM_Node_removeChild(node, newChild);

	if (NodeList_append(node->childNodes, newChild) == NULL) {
		return NULL;
	}

	if (node->firstChild == NULL) {
		node->firstChild = node->lastChild = newChild;
		newChild->previousSibling = NULL;
		newChild->nextSibling = NULL;
	} else {
		node->lastChild->nextSibling = newChild;
		newChild->previousSibling = node->lastChild;
		node->lastChild = newChild;
	}
    newChild->nextSibling = NULL;
    newChild->parentNode = node;

	if (addingDocumentElement) {
		node->u.Document.documentElement = newChild;
	}

    return newChild;
}
int
DOM_Node_hasChildNodes(const DOM_Node *node)
{
    return node != NULL && node->firstChild;
}
DOM_Node *
DOM_Node_cloneNode(DOM_Node *node, int deep)
{
    DOM_Node *clone = NULL;
	DOM_Node *ntmp, *ctmp;
	NodeEntry *e;

	if (node == NULL) {
		DOM_Exception = DOM_NULL_POINTER_ERR;
		return NULL;
	}

    switch(node->nodeType) {
        case DOM_ELEMENT_NODE:
			clone = DOM_Document_createElement(node->ownerDocument, node->nodeName);
			if (clone) {
				for (e = node->attributes->first; e != NULL; e = e->next) {
					if ((ctmp = DOM_Node_cloneNode(e->node, deep)) == NULL ||
										NodeList_append(clone->attributes, ctmp) == NULL) {
						DOM_Document_destroyNode(clone->ownerDocument, ctmp);
						DOM_Document_destroyNode(clone->ownerDocument, clone);
						return NULL;
					}
				}
			}
            break;
        case DOM_ATTRIBUTE_NODE:
			if ((clone = DOM_Document_createAttribute(node->ownerDocument, node->nodeName))) {
           		clone->u.Attr.specified = node->u.Attr.specified;
				free(clone->nodeValue);
           		clone->u.Attr.value = clone->nodeValue = DOM_String_dup(node->nodeValue);
				if (clone->u.Attr.value == NULL) {
					DOM_Exception = DOM_NO_MEMORY_ERR;
					return NULL;
				}
			}
            break;
		case DOM_COMMENT_NODE:
			clone = DOM_Document_createComment(node->ownerDocument, node->nodeValue);
			break;
		case DOM_TEXT_NODE:
			clone = DOM_Document_createTextNode(node->ownerDocument, node->nodeValue);
			break;
		case DOM_CDATA_SECTION_NODE:
			clone = DOM_Document_createCDATASection(node->ownerDocument, node->nodeValue);
			break;
        case DOM_DOCUMENT_NODE:
			clone = DOM_Implementation_createDocument(NULL, NULL, NULL);
			break;
		case DOM_DOCUMENT_FRAGMENT_NODE:
			clone = DOM_Document_createDocumentFragment(node->ownerDocument);
			break;
        case DOM_PROCESSING_INSTRUCTION_NODE:
			clone = DOM_Document_createProcessingInstruction(node->ownerDocument,
					node->u.ProcessingInstruction.target, node->u.ProcessingInstruction.data);
			break;
        case DOM_ENTITY_REFERENCE_NODE:
        case DOM_ENTITY_NODE:
        case DOM_DOCUMENT_TYPE_NODE:
        case DOM_NOTATION_NODE:
			DOM_Exception = DOM_NOT_SUPPORTED_ERR;
			return NULL;
    }

	if (clone && node->childNodes) {
		for (ntmp = node->firstChild; ntmp != NULL; ntmp = ntmp->nextSibling) {
			ctmp = DOM_Node_cloneNode(ntmp, deep);
			if (ctmp == NULL || DOM_Node_appendChild(clone, ctmp) == NULL) {
				DOM_Document_destroyNode(clone->ownerDocument, ctmp);
				DOM_Document_destroyNode(clone->ownerDocument, clone);
				return NULL;
			}
		}
	}


	//DBL: erase next line
	if (DOM_Node_hasChildNodes(node)) {}


    return clone;
}

