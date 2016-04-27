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

/* events.c
 */

#include <stdio.h>
#include <time.h>
#include "dom.h"

/* DOM_EventException
 */

unsigned short DOM_EventException;

/* DOM_DocumentEvent - Introduced in DOM Level 2
 */

void
DOM_DocumentEvent_destroyEvent(DOM_DocumentEvent *doc, DOM_Event *evt)
{
	if (doc && evt) {
		if (evt->type) {
			free(evt->type);
		}
		free(evt);
	}
}

DOM_Event *
DOM_DocumentEvent_createEvent(DOM_DocumentEvent *doc, const DOM_String *eventType)
{
	DOM_Event *evt;

	if (doc == NULL || eventType == NULL) {
		DOM_Exception = DOM_NULL_POINTER_ERR;
		return NULL;
	}
	if (DOM_String_cmp(eventType, "Events") == 0 ||
					DOM_String_cmp(eventType, "UIEvents") == 0 ||
					DOM_String_cmp(eventType, "KeyEvents") == 0) {
		evt = calloc(sizeof *evt, 1);
		if (evt == NULL) {
			DOM_Exception = DOM_NO_MEMORY_ERR;
			return NULL;
		}
	} else {
		DOM_Exception = DOM_NOT_SUPPORTED_ERR;
		return NULL;
	}
	return evt;
}

/* DOM_Event - Introduced in DOM Level 2
 */

void
DOM_Event_stopPropagation(DOM_Event *evt)
{
	if (evt) {
		evt->sp = 1;
	}
}
void
DOM_Event_preventDefault(DOM_Event *evt)
{
	if (evt && evt->cancelable) {
		evt->pd = 1;
	}
}
void
DOM_Event_initEvent(DOM_Event *evt, const DOM_String *eventTypeArg,
										int canBubbleArg, int cancelableArg)
{
	if (evt == NULL || eventTypeArg == NULL) {
		DOM_Exception = DOM_NULL_POINTER_ERR;
		return;
	}
	if (evt->type) {
		free(evt->type);
	}
	evt->type = DOM_String_dup(eventTypeArg);
	if (evt->type == NULL) {
		DOM_Exception = DOM_NO_MEMORY_ERR;
		return;
	}
	evt->bubbles = canBubbleArg;
	evt->cancelable = cancelableArg;
}

/* DOM_UIEvent
 */

void
DOM_UIEvent_initUIEvent(DOM_UIEvent *evt, DOM_String *typeArg,
										int canBubbleArg, int cancelableArg,
										DOM_AbstractView *viewArg, long detailArg)
{
	DOM_Event_initEvent(evt, typeArg, canBubbleArg, cancelableArg);
	if (DOM_Exception) {
		return;
	}
	evt->view = viewArg;
	evt->detail = detailArg;
}

/* DOM_EventTarget - Introduced in DOM Level 2
 */

void
DOM_EventTarget_addEventListener(DOM_EventTarget *target, const DOM_String *type,
										DOM_EventListener listener, int useCapture)
{
	ListenerEntry *e;
	unsigned int i, opos = -1;

	if (target == NULL || type == NULL || listener == NULL) {
		DOM_Exception = DOM_NULL_POINTER_ERR;
		return;
	}

	for (i = 0; i < target->listeners_len; i++) {
		e = target->listeners[i];                      /* skip duplicates */

		if (e == NULL) {
			if (opos == -1) {
				opos = i;             /* find open position for new entry */
			}
		} else if (e->listener_fn == listener && e->useCapture == useCapture &&
										DOM_String_cmp(e->type, type) == 0) {
			return;
		}
	}

	if ((e = malloc(sizeof *e)) == NULL ||
							(e->type = DOM_String_dup(type)) == NULL) {
		DOM_Exception = DOM_NO_MEMORY_ERR;
		free(e);
		return;
	}
	e->listener_fn = listener;
	e->useCapture = useCapture;

	if (opos == -1) {
		target->listeners = realloc(target->listeners,
										sizeof *e * (target->listeners_len + 1));
		if (target->listeners == NULL) {
			DOM_Exception = DOM_NO_MEMORY_ERR;
			free(e);
			return;
		}
		target->listeners[target->listeners_len++] = e;
	} else {
		target->listeners[opos] = e;
	}
}
void
DOM_EventTarget_removeEventListener(DOM_EventTarget *target, const DOM_String *type,
										DOM_EventListener listener, int useCapture)
{
	ListenerEntry *e;
	unsigned int i;

	if (target == NULL || type == NULL || listener == NULL) {
		DOM_Exception = DOM_NULL_POINTER_ERR;
		return;
	}

	for (i = 0; i < target->listeners_len; i++) {
		e = target->listeners[i];

		if (e && e->listener_fn == listener && e->useCapture == useCapture &&
										DOM_String_cmp(e->type, type) == 0) {
			target->listeners[i] = NULL;
			free(e->type);
			free(e);
			return;
		}
	}
}
int
DOM_EventTarget_dispatchEvent(DOM_EventTarget *target, DOM_Event *evt)
{
	DOM_EventTarget **targets, *t;
	ListenerEntry *e;
	unsigned int tcount, i, j, lcount;

	if (target == NULL || evt == NULL) {
		DOM_Exception = DOM_NULL_POINTER_ERR;
		return 1;
	}

	if (evt->type == NULL || *evt->type == '\0') {
		DOM_EventException = DOM_UNSPECIFIED_EVENT_TYPE_ERR;
		return 1;
	}

	evt->target = target;							/* post-initialization */
	evt->timeStamp = time(NULL);
	if (evt->timeStamp == (time_t)-1) {
		DOM_Exception = DOM_SYSTEM_ERR;
		return 1;
	}
	evt->sp = 0;
	evt->pd = 0;

	tcount = 0;									/* count targets/ancestors */
	for (t = target->parentNode; t; t = t->parentNode) {
		tcount++;
	}
	if (tcount) {
		targets = malloc(sizeof *targets * tcount);
		if (targets == NULL) {
			DOM_Exception = DOM_NO_MEMORY_ERR;
			return 1;
		}
	}
	i = tcount;						/* save state of tree in targets array */
	for (t = target->parentNode; t; t = t->parentNode) {
		targets[--i] = t;
	}

	/* Trigger capturers
	 */
	evt->eventPhase = DOM_EVENT_CAPTURING_PHASE;
	for (i = 0; i < tcount && evt->sp == 0; i++) {
		DOM_EventListener cpy_of_listener_fns[targets[i]->listeners_len];
                                                          /* use stack ok? */
		t = targets[i];
		lcount = t->listeners_len;              /* copy listener functions */
		for (j = 0; j < lcount; j++) {
			e = t->listeners[j];
			cpy_of_listener_fns[j] = e ? e->listener_fn : NULL;
		}

		evt->currentTarget = t;
		for (j = 0; j < lcount; j++) {
			e = t->listeners[j]; /* If the entry is NULL, the listener has
                          * since been removed and is therefore skipped. If it
                          * is not NULL but the listener functions do not match,
                          * it was removed but another listener was added in
                          * its place and therefore should be skipped. However,
                          * if a listener is removed but then added again
                          * while the current set is still being processed
                          * there is a chance that it will be tiggered if it
                          * meets the criteria tested below. Regardless of
                          * the slim chances of this occuring (have to be the
                          * same listener function added to the same array
                          * index ... etc), it will need to be fixed.
                          */
			if (e && e->listener_fn == cpy_of_listener_fns[j] &&
									e->useCapture &&
									DOM_String_cmp(e->type, evt->type) == 0) {
				cpy_of_listener_fns[j](evt); /* invoke the listener function */
			}
		}
	}

	/* Trigger regular listeners
 	 */
	evt->eventPhase = DOM_EVENT_AT_TARGET;
	if (target->listeners_len && evt->sp == 0) {
		DOM_EventListener cpy_of_listener_fns[target->listeners_len];

		lcount = target->listeners_len;
		for (j = 0; j < lcount; j++) {
			e = target->listeners[j];
			cpy_of_listener_fns[j] = e ? e->listener_fn : NULL;
		}

		evt->currentTarget = target;
		for (j = 0; j < lcount; j++) {
			e = target->listeners[j];
			if (e && cpy_of_listener_fns[j] && e->useCapture == 0 &&
									DOM_String_cmp(e->type, evt->type) == 0) {
				cpy_of_listener_fns[j](evt);
			}
		}
	}

	/* Trigger bubblers
 	 */
	evt->eventPhase = DOM_EVENT_BUBBLING_PHASE;
	i = tcount;
	while (i-- && evt->bubbles && evt->sp == 0) {
		DOM_EventListener cpy_of_listener_fns[targets[i]->listeners_len];

		t = targets[i];
		lcount = t->listeners_len;
		for (j = 0; j < lcount; j++) {
			e = t->listeners[j];
			cpy_of_listener_fns[j] = e ? e->listener_fn : NULL;
		}

		evt->currentTarget = t;
		for (j = 0; j < lcount; j++) {
			e = t->listeners[j];
			if (e && cpy_of_listener_fns[j] && e->useCapture == 0 &&
									DOM_String_cmp(e->type, evt->type) == 0) {
				cpy_of_listener_fns[j](evt);
			}
		}
	}

	if (targets) {
		free(targets);
	}
	return evt->pd;
}

/* DOM_KeyEvent - Introduced in DOM Level 3
 * http://www.w3.org/TR/2001/WD-DOM-Level-3-Events-20010410/events.html
 */

int
DOM_KeyEvent_checkModifier(unsigned long modifier)
{
	return 0;
}
void
DOM_KeyEvent_initKeyEvent(DOM_KeyEvent *evt, const DOM_String *typeArg,
										int canBubbleArg, int cancelableArg,
										DOM_AbstractView *viewArg, unsigned short detailArg,
										DOM_String *outputStringArg,
										unsigned long keyValArg,
										unsigned long virtKeyValArg,
										int inputGeneratedArg,
										int numPadArg)
{
}
void
DOM_KeyEvent_initModifier(unsigned long modifier, int value)
{
}

/* MutationEvent
 */

void
DOM_MutationEvent_initMutationEvent(DOM_MutationEvent *evt, DOM_String *typeArg,
										int canBubbleArg, int cancelableArg,
										DOM_Node *relatedNodeArg,
										DOM_String *prevValueArg,
										DOM_String *newValueArg,
										DOM_String *attrNameArg,
										unsigned short attrChangeArg)
{
}
