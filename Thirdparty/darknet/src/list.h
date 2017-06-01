#ifndef list_H
#define list_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct node {
    void *val;
    struct node *next;
    struct node *prev;
} node;

typedef struct listc {
    int size;
    node *front;
    node *back;
} listc;

listc *make_list();
int list_find(listc *l, void *val);

void list_insert(listc *, void *);

void **list_to_array(listc *l);

void free_list(listc *l);
void free_listontents(listc *l);

#ifdef __cplusplus
}
#endif

#endif
