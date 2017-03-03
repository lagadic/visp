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

/* expatls.c - DOM_DocumentLS
 */
#include <limits.h>
#include <ctype.h>
#include <stdio.h>
#include "xmlparse.h"
#include "stack.h"
#include "dom.h"

const char *SP = "                                ";
#define INDENT(n) ((n) > 8 ? SP : (SP + (32 - (n) * 4)))
#define BUF_SIZ 8192

void
start_fn(void *userData, const XML_Char *name, const XML_Char **atts)
{
	struct stack *stk = (struct stack *) userData;
	DOM_Node *parent, *child;
	int i;

	if (stk == NULL || name == NULL || atts == NULL) {
		DOM_Exception = DOM_NULL_POINTER_ERR;
		return;
	}

	parent = (DOM_Node *) stack_peek(stk);
	if (parent == NULL) {
		DOM_Exception = DOM_SYSTEM_ERR;
		return;
	}
	child = DOM_Document_createElement(parent->ownerDocument, name);
	if (child == NULL) {
		return;
	}
	for (i = 0; atts[i]; i += 2) {
		DOM_Element_setAttribute(child, atts[i], atts[i + 1]);
		if (DOM_Exception) {
			return;
		}
	}
	if (DOM_Node_appendChild(parent, child) == NULL) {
		return;
	}
	if (stack_push(stk, child) == 0) {
		DOM_Exception = DOM_SYSTEM_ERR;
	}
}
void
end_fn(void *userData, const XML_Char *name)
{
	stack_pop((struct stack *)userData);
}
void
chardata_fn(void *userData, const XML_Char *s, int len)
{
	struct stack *stk = (struct stack *) userData;
	DOM_String *str;
	DOM_Text *tex;
	DOM_Node *parent;

	if (stk == NULL || s == NULL || len == 0) {
		DOM_Exception = DOM_NULL_POINTER_ERR;
		return;
	}

	parent = (DOM_Node *) stack_peek(stk);
	if (parent == NULL) {
		DOM_Exception = DOM_SYSTEM_ERR;
		return;
	}
	if ((str = (DOM_String *) malloc(len + 1)) == NULL) {
		DOM_Exception = DOM_NO_MEMORY_ERR;
		return;
	}
	memcpy(str, s, len);
	str[len] = '\0';
	tex = DOM_Document_createTextNode(parent->ownerDocument, str);
	free(str);
	if (tex == NULL) {
		return;
	}

	DOM_Node_appendChild(parent, tex);
	if (DOM_Exception) {
		DOM_Document_destroyNode(parent->ownerDocument, tex);
	}
}
void
comment_fn(void *userData, const XML_Char *s)
{
	struct stack *stk = (struct stack *) userData;
	DOM_Comment *com;
	DOM_Node *parent;

	parent = (DOM_Node *) stack_peek(stk);
	if (parent == NULL) {
		DOM_Exception = DOM_SYSTEM_ERR;
		return;
	}
	if ((com = DOM_Document_createComment(parent->ownerDocument, s))) {
		DOM_Node_appendChild(parent, com);
		if (DOM_Exception) {
			DOM_Document_destroyNode(parent->ownerDocument, com);
		}
	}
}
void
processing_fn(void *userData, const XML_Char *target, const XML_Char *data)
{
	struct stack *stk = (struct stack *) userData;
	DOM_ProcessingInstruction *pi;
	DOM_Node *parent;

	parent = (DOM_Node *) stack_peek(stk);
	if (parent == NULL) {
		DOM_Exception = DOM_SYSTEM_ERR;
		return;
	}
	if ((pi = DOM_Document_createProcessingInstruction(parent->ownerDocument, target, data))) {
		DOM_Node_appendChild(parent, pi);
		if (DOM_Exception) {
			DOM_Document_destroyNode(parent->ownerDocument, pi);
		}
	}
}
int
DOM_DocumentLS_load(DOM_Document *doc, const DOM_String *uri)
{
	FILE *fd;
	XML_Parser p;
	struct stack *stk;
	size_t n;	// was ssize_t (DBL)
	void *buf;
	int ret, done;

	DOM_Exception=DOM_NO_ERR; //line added by DBL

	if (doc == NULL || uri == NULL) {
		DOM_Exception = DOM_NULL_POINTER_ERR;
		return 0;
	}

	fd = fopen(uri, "r");
	if (fd == NULL) {
		DOM_Exception = DOM_SYSTEM_ERR;
		return 0;
	}

	p = XML_ParserCreate(NULL);
	if (p == NULL) {
		DOM_Exception = DOM_XML_PARSER_ERR;
		fclose(fd);
		return 0;
	}

	stk = stack_new(INT_MAX);
	if (stk == NULL || stack_push(stk, doc) == 0) {
		DOM_Exception = DOM_SYSTEM_ERR;
		XML_ParserFree(p);
		fclose(fd);
		stack_del(stk, NULL);
		return 0;
	}

	XML_SetElementHandler(p, start_fn, end_fn);
	XML_SetCharacterDataHandler(p, chardata_fn);
	XML_SetCommentHandler(p, comment_fn);
	XML_SetProcessingInstructionHandler(p , processing_fn);
	XML_SetUserData(p, stk);

	ret = 1;
	for ( ;; ) {
		if ((buf = XML_GetBuffer(p, BUF_SIZ)) == NULL) {
			DOM_Exception = DOM_NO_MEMORY_ERR;
			ret = 0;
			break;
		}
		if ((n = fread(buf, 1, BUF_SIZ, fd)) == 0 && ferror(fd)) {
			DOM_Exception = DOM_SYSTEM_ERR;
			ret = 0;
			break;
		}
		if (XML_ParseBuffer(p, (int) n, (done = feof(fd))) == 0 || DOM_Exception) {
			if (DOM_Exception == 0) {
				DOM_Exception = DOM_XML_PARSER_ERR;
			}
			ret = 0;
			break;
		}
		if (done) {
			break;
		}
	}

	stack_del(stk, NULL);
	XML_ParserFree(p);
	fclose(fd);

	return ret;
}


void
DocumentLS_save(FILE *fd, const DOM_Node *node, int indent)
{
	NodeEntry *e;
	DOM_Node *c;

    if (fd == NULL || node == NULL) {
		return;
    }

    switch (node->nodeType) {
        case DOM_ELEMENT_NODE:
            fprintf(fd, "%s<%s", INDENT(indent), node->nodeName);
			for (e = node->attributes->first; e != NULL; e = e->next) {
				fprintf(fd, " %s=\"%s\"", e->node->nodeName, e->node->nodeValue);
			}
			if (DOM_Node_hasChildNodes(node)) {
				fprintf(fd, ">\n");
				for (c = node->firstChild; c != NULL; c = c->nextSibling) {
					DocumentLS_save(fd, c, indent + 1);
            	}
            	fprintf(fd, "%s</%s>\n", INDENT(indent), node->nodeName);
			} else {
				fprintf(fd, "/>\n");
			}
            break;
        case DOM_ATTRIBUTE_NODE:
            break;
        case DOM_TEXT_NODE:
            fprintf(fd, "%s\n", node->nodeValue);
            break;
        case DOM_CDATA_SECTION_NODE:
            break;
        case DOM_ENTITY_REFERENCE_NODE:
            break;
        case DOM_ENTITY_NODE:
            break;
        case DOM_PROCESSING_INSTRUCTION_NODE:
            break;
        case DOM_COMMENT_NODE:
			fprintf(fd, "%s<!--%s-->\n", INDENT(indent), node->nodeValue);
            break;
        case DOM_DOCUMENT_NODE:
            DocumentLS_save(fd, node->u.Document.documentElement, 0);
            break;
        case DOM_DOCUMENT_TYPE_NODE:
            break;
        case DOM_DOCUMENT_FRAGMENT_NODE:
            break;
        case DOM_NOTATION_NODE:
            break;
    }
}
int
DOM_DocumentLS_save(DOM_Document *doc, const DOM_String *uri, const DOM_Node *node)
{
	FILE *fd;

	if (doc == NULL && node == NULL) {
		DOM_Exception = DOM_NULL_POINTER_ERR;
		return 0;
	}

	fd = fopen(uri, "w");
	if (fd) {
		DocumentLS_save(fd, doc ? doc : node, 0);
		fclose(fd);
		return 1;
	}

	return 0;
}
