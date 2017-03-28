#include "f2c.h"
#include "fio.h"
#include "fmt.h"
#ifdef __cplusplus
extern "C" {
#endif

extern icilist *f__svic;
extern char *f__icptr;

 static int
mv_cur(Void)	/* shouldn't use fseek because it insists on calling fflush */
		/* instead we know too much about stdio */
{
	int cursor = f__cursor;
	f__cursor = 0;
	if(f__external == 0) {
		if(cursor < 0) {
			if(f__hiwater < f__recpos)
				f__hiwater = f__recpos;
			f__recpos += cursor;
			f__icptr += cursor;
			if(f__recpos < 0)
				err(f__elist->cierr, 110, "left off");
		}
		else if(cursor > 0) {
			if(f__recpos + cursor >= f__svic->icirlen)
				err(f__elist->cierr, 110, "recend");
			if(f__hiwater <= f__recpos)
				for(; cursor > 0; cursor--)
					(*f__putn)(' ');
			else if(f__hiwater <= f__recpos + cursor) {
				cursor -= f__hiwater - f__recpos;
				f__icptr += f__hiwater - f__recpos;
				f__recpos = f__hiwater;
				for(; cursor > 0; cursor--)
					(*f__putn)(' ');
			}
			else {
				f__icptr += cursor;
				f__recpos += cursor;
			}
		}
		return(0);
	}
	if (cursor > 0) {
		if(f__hiwater <= f__recpos)
			for(;cursor>0;cursor--) (*f__putn)(' ');
		else if(f__hiwater <= f__recpos + cursor) {
			cursor -= f__hiwater - f__recpos;
			f__recpos = f__hiwater;
			for(; cursor > 0; cursor--)
				(*f__putn)(' ');
		}
		else {
			f__recpos += cursor;
		}
	}
	else if (cursor < 0)
	{
		if(cursor + f__recpos < 0)
			err(f__elist->cierr,110,"left off");
		if(f__hiwater < f__recpos)
			f__hiwater = f__recpos;
		f__recpos += cursor;
	}
	return(0);
}

 static int
#ifdef KR_headers
wrt_Z(n,w,minlen,len) Uint *n; int w, minlen; ftnlen len;
#else
wrt_Z(Uint *n, int w, int minlen, ftnlen len)
#endif
{
	register char *s, *se;
	register int i, w1;
	static int one = 1;
	static char hex[] = "0123456789ABCDEF";
	s = (char *)n;
	--len;
	if (*(char *)&one) {
		/* little endian */
		se = s;
		s += len;
		i = -1;
		}
	else {
		se = s + len;
		i = 1;
		}
	for(;; s += i)
		if (s == se || *s)
			break;
	w1 = (i*(se-s) << 1) + 1;
	if (*s & 0xf0)
		w1++;
	if (w1 > w)
		for(i = 0; i < w; i++)
			(*f__putn)('*');
	else {
		if ((minlen -= w1) > 0)
			w1 += minlen;
		while(--w >= w1)
			(*f__putn)(' ');
		while(--minlen >= 0)
			(*f__putn)('0');
		if (!(*s & 0xf0)) {
			(*f__putn)(hex[*s & 0xf]);
			if (s == se)
				return 0;
			s += i;
			}
		for(;; s += i) {
			(*f__putn)(hex[*s >> 4 & 0xf]);
			(*f__putn)(hex[*s & 0xf]);
			if (s == se)
				break;
			}
		}
	return 0;
	}

 static int
#ifdef KR_headers
wrt_I(n,w,len, base) Uint *n; ftnlen len; register int base;
#else
wrt_I(Uint *n, int w, ftnlen len, register int base)
#endif
{	int ndigit,sign,spare,i;
	longint x;
	char *ans;
	if(len==sizeof(integer)) x=n->il;
	else if(len == sizeof(char)) x = n->ic;
#ifdef Allow_TYQUAD
	else if (len == sizeof(longint)) x = n->ili;
#endif
	else x=n->is;
	ans=f__icvt(x,&ndigit,&sign, base);
	spare=w-ndigit;
	if(sign || f__cplus) spare--;
	if(spare<0)
		for(i=0;i<w;i++) (*f__putn)('*');
	else
	{	for(i=0;i<spare;i++) (*f__putn)(' ');
		if(sign) (*f__putn)('-');
		else if(f__cplus) (*f__putn)('+');
		for(i=0;i<ndigit;i++) (*f__putn)(*ans++);
	}
	return(0);
}
 static int
#ifdef KR_headers
wrt_IM(n,w,m,len,base) Uint *n; ftnlen len; int base;
#else
wrt_IM(Uint *n, int w, int m, ftnlen len, int base)
#endif
{	int ndigit,sign,spare,i,xsign;
	longint x;
	char *ans;
	if(sizeof(integer)==len) x=n->il;
	else if(len == sizeof(char)) x = n->ic;
#ifdef Allow_TYQUAD
	else if (len == sizeof(longint)) x = n->ili;
#endif
	else x=n->is;
	ans=f__icvt(x,&ndigit,&sign, base);
	if(sign || f__cplus) xsign=1;
	else xsign=0;
	if(ndigit+xsign>w || m+xsign>w)
	{	for(i=0;i<w;i++) (*f__putn)('*');
		return(0);
	}
	if(x==0 && m==0)
	{	for(i=0;i<w;i++) (*f__putn)(' ');
		return(0);
	}
	if(ndigit>=m)
		spare=w-ndigit-xsign;
	else
		spare=w-m-xsign;
	for(i=0;i<spare;i++) (*f__putn)(' ');
	if(sign) (*f__putn)('-');
	else if(f__cplus) (*f__putn)('+');
	for(i=0;i<m-ndigit;i++) (*f__putn)('0');
	for(i=0;i<ndigit;i++) (*f__putn)(*ans++);
	return(0);
}
 static int
#ifdef KR_headers
wrt_AP(s) char *s;
#else
wrt_AP(char *s)
#endif
{	char quote;
	int i;

	if(f__cursor && (i = mv_cur()))
		return i;
	quote = *s++;
	for(;*s;s++)
	{	if(*s!=quote) (*f__putn)(*s);
		else if(*++s==quote) (*f__putn)(*s);
		else return(1);
	}
	return(1);
}
 static int
#ifdef KR_headers
wrt_H(a,s) char *s;
#else
wrt_H(int a, char *s)
#endif
{
	int i;

	if(f__cursor && (i = mv_cur()))
		return i;
	while(a--) (*f__putn)(*s++);
	return(1);
}

 int
#ifdef KR_headers
wrt_L(n,len, sz) Uint *n; ftnlen sz;
#else
wrt_L(Uint *n, int len, ftnlen sz)
#endif
{	int i;
	long x;
	if(sizeof(long)==sz) x=n->il;
	else if(sz == sizeof(char)) x = n->ic;
	else x=n->is;
	for(i=0;i<len-1;i++)
		(*f__putn)(' ');
	if(x) (*f__putn)('T');
	else (*f__putn)('F');
	return(0);
}
 static int
#ifdef KR_headers
wrt_A(p,len) char *p; ftnlen len;
#else
wrt_A(char *p, ftnlen len)
#endif
{
	while(len-- > 0) (*f__putn)(*p++);
	return(0);
}
 static int
#ifdef KR_headers
wrt_AW(p,w,len) char * p; ftnlen len;
#else
wrt_AW(char * p, int w, ftnlen len)
#endif
{
	while(w>len)
	{	w--;
		(*f__putn)(' ');
	}
	while(w-- > 0)
		(*f__putn)(*p++);
	return(0);
}

 static int
#ifdef KR_headers
wrt_G(p,w,d,e,len) ufloat *p; ftnlen len;
#else
wrt_G(ufloat *p, int w, int d, int e, ftnlen len)
#endif
{	double up = 1,x;
	int i=0,oldscale,n,j;
	x = len==sizeof(real)?p->pf:p->pd;
	if(x < 0 ) x = -x;
	if(x<.1) {
		if (x != 0.)
			return(wrt_E(p,w,d,e,len));
		i = 1;
		goto have_i;
		}
	for(;i<=d;i++,up*=10)
	{	if(x>=up) continue;
 have_i:
		oldscale = f__scale;
		f__scale = 0;
		if(e==0) n=4;
		else	n=e+2;
		i=wrt_F(p,w-n,d-i,len);
		for(j=0;j<n;j++) (*f__putn)(' ');
		f__scale=oldscale;
		return(i);
	}
	return(wrt_E(p,w,d,e,len));
}

 int
#ifdef KR_headers
w_ed(p,ptr,len) struct syl *p; char *ptr; ftnlen len;
#else
w_ed(struct syl *p, char *ptr, ftnlen len)
#endif
{
	int i;

	if(f__cursor && (i = mv_cur()))
		return i;
	switch(p->op)
	{
	default:
		fprintf(stderr,"w_ed, unexpected code: %d\n", p->op);
		sig_die(f__fmtbuf, 1);
	case I:	return(wrt_I((Uint *)ptr,p->p1,len, 10));
	case IM:
		return(wrt_IM((Uint *)ptr,p->p1,p->p2.i[0],len,10));

		/* O and OM don't work right for character, double, complex, */
		/* or doublecomplex, and they differ from Fortran 90 in */
		/* showing a minus sign for negative values. */

	case O:	return(wrt_I((Uint *)ptr, p->p1, len, 8));
	case OM:
		return(wrt_IM((Uint *)ptr,p->p1,p->p2.i[0],len,8));
	case L:	return(wrt_L((Uint *)ptr,p->p1, len));
	case A: return(wrt_A(ptr,len));
	case AW:
		return(wrt_AW(ptr,p->p1,len));
	case D:
	case E:
	case EE:
		return(wrt_E((ufloat *)ptr,p->p1,p->p2.i[0],p->p2.i[1],len));
	case G:
	case GE:
		return(wrt_G((ufloat *)ptr,p->p1,p->p2.i[0],p->p2.i[1],len));
	case F:	return(wrt_F((ufloat *)ptr,p->p1,p->p2.i[0],len));

		/* Z and ZM assume 8-bit bytes. */

	case Z: return(wrt_Z((Uint *)ptr,p->p1,0,len));
	case ZM:
		return(wrt_Z((Uint *)ptr,p->p1,p->p2.i[0],len));
	}
}

 int
#ifdef KR_headers
w_ned(p) struct syl *p;
#else
w_ned(struct syl *p)
#endif
{
	switch(p->op)
	{
	default: fprintf(stderr,"w_ned, unexpected code: %d\n", p->op);
		sig_die(f__fmtbuf, 1);
	case SLASH:
		return((*f__donewrec)());
	case T: f__cursor = p->p1-f__recpos - 1;
		return(1);
	case TL: f__cursor -= p->p1;
		if(f__cursor < -f__recpos)	/* TL1000, 1X */
			f__cursor = -f__recpos;
		return(1);
	case TR:
	case X:
		f__cursor += p->p1;
		return(1);
	case APOS:
		return(wrt_AP(p->p2.s));
	case H:
		return(wrt_H(p->p1,p->p2.s));
	}
}
#ifdef __cplusplus
}
#endif
