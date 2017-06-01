#include <stdlib.h>
#include <string.h>
#include "list.h"

listc *make_list()
{
    listc *l = (listc*)malloc(sizeof(listc));
	l->size = 0;
	l->front = 0;
	l->back = 0;
	return l;
}

/*
void transfer_node(listc *s, listc *d, node *n)
{
    node *prev, *next;
    prev = n->prev;
    next = n->next;
    if(prev) prev->next = next;
    if(next) next->prev = prev;
    --s->size;
    if(s->front == n) s->front = next;
    if(s->back == n) s->back = prev;
}
*/

void *list_pop(listc *l){
    if(!l->back) return 0;
    node *b = l->back;
    void *val = b->val;
    l->back = b->prev;
    if(l->back) l->back->next = 0;
    free(b);
    --l->size;
    
    return val;
}

void list_insert(listc *l, void *val)
{
	node *new_ = (node*)malloc(sizeof(node));
	new_->val = val;
	new_->next = 0;

	if(!l->back){
		l->front = new_;
		new_->prev = 0;
	}else{
		l->back->next = new_;
		new_->prev = l->back;
	}
	l->back = new_;
	++l->size;
}

void free_node(node *n)
{
	node *next;
	while(n) {
		next = n->next;
		free(n);
		n = next;
	}
}

void free_list(listc *l)
{
	free_node(l->front);
	free(l);
}

void free_listontents(listc *l)
{
	node *n = l->front;
	while(n){
		free(n->val);
		n = n->next;
	}
}

void **list_to_array(listc *l)
{
    void **a = (void**)calloc(l->size, sizeof(void*));
    int count = 0;
    node *n = l->front;
    while(n){
        a[count++] = n->val;
        n = n->next;
    }
    return a;
}
