/*** sort/sumsol */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <assert.h>

#define MAX_EVT_SIZE 4000000

struct TMidas_EVENT_HEADER {
  uint16_t fEventId;      ///< event id
  uint16_t fTriggerMask;  ///< event trigger mask
  uint32_t fSerialNumber; ///< event serial number
  uint32_t fTimeStamp;    ///< event timestamp in seconds
  uint32_t fDataSize;     ///< event size in bytes
};

struct TMidas_BANK_HEADER {
  uint32_t fDataSize;
  uint32_t fFlags;
};

struct TMidas_BANK {
  char fName[4];      ///< bank name
  uint16_t fType;     ///< type of data (see midas.h TID_xxx)
  uint16_t fDataSize;
};

struct TMidas_BANK32 {
  char fName[4];      ///< bank name
  uint32_t fType;     ///< type of data (see midas.h TID_xxx)
  uint32_t fDataSize;
};

#define L_LEN 100000
struct hit {
  int sn;
  int evt_id;
  int iname;
  int b_id;
  int ch_id;
  int val;
};

#define EVT_ADC 1  // midas event ids
#define EVT_TDC 2

struct hit adc_l[L_LEN];
struct hit tdc_l[L_LEN];
int adc_cnt = 0, tdc_cnt = 0;

int evt_num = 0;

int idlist[100][2];
int id_num = 0, id_found;

int first_tdc = 1;
char tdc_names[7][5] = {"ICT_", "YDT_", "YUT_", "SUT_" ,"SD2T" ,"SD1T", "VTRT"};

void mergewrite(FILE *fou) {
  struct hit *h;
  static int first = 1;
  int i;
  char *c;

  printf("mergewrite - adc_cnt = %d\n", adc_cnt);
  assert(fou != 0);
  if (first == 1) {
    fprintf(fou, "evt_num,type,name,b_id,ch_id,val\n");
    first = 0;
  }

  for (i = 0; i < adc_cnt; i++) {
    h = adc_l + i;
    c = (char *) &(h->iname);
    fprintf(fou, "%d,%d,%c%c%c%c,%d,%d,%d\n", h->sn, h->evt_id, c[0], c[1], c[2], c[3], h->b_id, h->ch_id, h->val);
  }
}

void v1190(char *name, char *p, int num) {

  unsigned int *pa;
  struct hit *h;
  int tdc, v_ch, m_ch, i, val;
  static int first = 1;
  int num_hits;
 
  num_hits = 0;
  pa = (unsigned int *) p;
  for (i = 0; i < num / 4; i++) {
    switch (*(pa + i) & 0xf8000000) {
    case 0x40000000:	// global header
      //printf("global hdr\n");
      break;
    case 0x80000000:	// global header
      //printf("global trailer\n");
      break;
    case 0x08000000:	// tdc header
      //printf("tdc hdr\n");
      tdc  = 0x3 & (*(pa + i) >> 24);
      break;
    case 0x00000000:	// tdc meas
      //printf("tdc meas\n");
      v_ch = 0x3f & (*(pa + i) >> 19);
      m_ch = (tdc / 2) * 64 + v_ch;
      val = *(pa + i) & 0x7FFFF;
      h = tdc_l + adc_cnt;
      h->sn = evt_num;
      h->evt_id = EVT_TDC;
      num_hits++;
      //h->iname = iname;
      //h->b_id = b_id;
      //fprintf(fv1190, "%d,%c%c%c%c,%d,%d,%d\n", evt_num, name[0], name[1], name[2], name[3], v_ch, m_ch, val);
    case 0x18000000:	// tdc trailer
      //printf("tdc trailer\n");
      break;
    case 0x20000000:	// tdc error
      //printf("tdc error\n");
      break;
    default:
      //printf("unknown id .. 0x%x\n", *(pa + i) & 0xf8000000);
      ;
    }
  }
  //printf("end midas event\n");
  //fprintf(stderr, "%d\n", num_hits);
  //exit(1);
}

int mesy(int iname, char *p, int num, int *b_mask) {
  unsigned int *pa;
  struct hit *h;
  int b_id, ch_id, val, num_words, i;
  pa = (unsigned int *) p;
  static int first = 1;
  int num_hits;
  int sub_id;

  num_hits = 0;
  sub_id = 0;
  b_id = -1;
  for (i = 0; i < num / 4; i++) {
    switch (*(pa + i) & 0xc0000000) {
    case 0x40000000:
      b_id = (*(pa + i) >> 16) & 0xff;
      assert((*b_mask & (1 << b_id)) == 0);
      *b_mask |= (1 << b_id);
      num_words = *(pa + i) & 0xfff;
      //printf("hdr, evt_num = %d, b_id = %d, adc_res = %d, numWords = %d\n", evt_num, (*(pa + i) >> 16) & 0xff, (*(pa + i) >> 12) & 0x7, num_words);
      break;
    case 0x00000000:
      if (1) {	// 4 + trailer 
        ch_id = (*(pa + i) >> 18) & 0x1f;
        val = *(pa + i) & 0xff;
        h = adc_l + adc_cnt;
        h->sn = evt_num;
        h->evt_id = EVT_ADC;
        h->iname = iname;
        h->b_id = b_id;
        h->ch_id = (*(pa + i) >> 18) & 0x1f;
        h->val = *(pa + i) & 0xff;
        adc_cnt++;
        num_hits++;
      }
      //printf("data, evt_num = %d, sub_id = %d, board = %d, ch = %d, val = %d\n", evt_num, sub_id, h->b_id, h->ch_id, h->val);
      break;
    case 0xc0000000:
      //printf("eoe, ts = %d\n", 0x3fffffff & *(pa + i));
      break;
    default:
      ;//printf("something is wrong\n");
    }
  }
  return num_hits;
}

int procSubEvt_tdc(char *buf, int size, int evtNum, int sn, FILE *ftdc) {
  int *iname, *ibuf;
  int subEvtNum, i, j;
  char *s, *ss;
  struct TMidas_BANK32 *a;

  if (first_tdc) {
    fprintf(ftdc, "evt_num, sub_evt_num, s_num, tag, sub_evt_type, sub_evt_sz\n");
    first_tdc = 0;
  }
  subEvtNum = 0;
  ibuf = (int *) buf;
  for (i = 0; i < size / 4; i++) {
/*
    ss = (char *) buf + 4 * i;
    printf("%c%c%c%c\n", ss[0], ss[1], ss[2], ss[3]);
*/
    for (j = 0; j < 6; j++) {
      iname = (int *) &tdc_names[j][0];
      if (*iname == *(ibuf + i)) {
        s = (char *) (ibuf + i);
        a = (struct TMidas_BANK32 *) s;
//        fprintf(ftdc, "%d, %d, %c%c%c%c, %d, %d\n", evtNum, subEvtNum, s[0], s[1], s[2], s[3], a->fType, a->fDataSize);
        fprintf(ftdc, "%d, %d, %d, %c%c%c%c, %d, %d\n", evtNum, subEvtNum, sn, s[0], s[1], s[2], s[3], a->fType, a->fDataSize);
        subEvtNum += 1;
        v1190((char *) iname, s + sizeof(struct TMidas_BANK32), a->fDataSize);
      }
    }
  }
  return 0;
}

int first_adc = 1;
char adc_names[7][5] = {"ICA_", "SD2A", "SD1A", "YDA_","SUA_", "YUA_", "VTRA"};

int procSubEvt_adc(char *buf, int size, int evtNum, int sn, FILE *fadc) {
  int *iname, *ibuf;
  int subEvtNum, i, j;
  char *s, *ss;
  struct TMidas_BANK32 *a;
  int tot_hits, b_mask;
  tot_hits = 0;
  b_mask = 0;   // board mask

  if (first_adc) {
    fprintf(fadc, "evt_num, sub_evt_num, s_num, tag, sub_evt_type, sub_evt_sz\n");
    first_adc = 0;
  }
  subEvtNum = 0;
  ibuf = (int *) buf;
  for (i = 0; i < size / 4; i++) {
    for (j = 0; j < 6; j++) {
      iname = (int *) &adc_names[j][0];
      if (*iname == *(ibuf + i)) {
        s = (char *) (ibuf + i);
        a = (struct TMidas_BANK32 *) s;
        fprintf(fadc, "%d, %d, %d, %c%c%c%c, %d, %d\n", evtNum, subEvtNum, sn, s[0], s[1], s[2], s[3], a->fType, a->fDataSize);
        subEvtNum += 1;
        tot_hits += mesy(*iname, s + sizeof(struct TMidas_BANK32), a->fDataSize, &b_mask);
      }
    }
  }
  assert(b_mask == 0xffff);
  //fprintf(stderr, "%d,%d\n", tot_hits, size);
  return 0;
}

int main() {
  struct TMidas_EVENT_HEADER a;
  struct TMidas_BANK_HEADER *b;
  struct TMidas_BANK *c;
  struct TMidas_BANK32 *d;
  
  char *buf;
  FILE *fin, *fou, *fou2, *fou3, *fou4;
  int id, numEvts = 0, num, m32true, i;

  printf("**sumsol..\n");
  memset(idlist, 0, sizeof(int) * 100 * 2);
  buf = malloc(MAX_EVT_SIZE);

  fin = fopen("iris_000046080000.mid", "r");
  assert(fin != 0);
  fou = fopen("evtsize.csv", "w");
  fou2 = fopen("subevt_tdc.csv", "w");
  fou3 = fopen("subevt_adc.csv", "w");
  fou4 = fopen("merge.csv", "w");
  fprintf(fou, "evt_id, size, bsize, flag32, ssize, stype, tag\n");
  adc_cnt = tdc_cnt = 0;
  while( fread(&a, sizeof(struct TMidas_EVENT_HEADER), 1, fin) == 1) {
    // if ( (numEvts++ % 1000) == 0) { printf("numEvts = %d\n", numEvts);}
    //printf("sn = %d, t = %d\n", a.fSerialNumber, a.fTimeStamp);
    numEvts++;
    id = a.fEventId;
    num = fread(buf, a.fDataSize, 1, fin); 
    switch (id) {
    case 0x8000:
      printf("beginning of run tag\n");
      fprintf(fou, "%d, %d\n", a.fEventId, a.fDataSize);
      break;
    case 0x8001:
      printf("end of run tag\n");
      fprintf(fou, "%d, %d\n", a.fEventId, a.fDataSize);
      break;
    case 1:
      (void) procSubEvt_adc(buf + sizeof(struct TMidas_BANK_HEADER), a.fDataSize - sizeof(struct TMidas_BANK_HEADER), numEvts, a.fSerialNumber, fou3);
      evt_num++;
      break;
    case 2:
      (void) procSubEvt_tdc(buf + sizeof(struct TMidas_BANK_HEADER), a.fDataSize - sizeof(struct TMidas_BANK_HEADER), numEvts, a.fSerialNumber, fou2);
      evt_num++;
      break;
    default:
      break;
    }

    /* flush adc, tdc hit lists is buffers full */
    if (adc_cnt > L_LEN - 256 || tdc_cnt > L_LEN - 256) {
      mergewrite(fou4);
      adc_cnt = tdc_cnt = 0;
    }

    for (i = 0, id_found = 0; i < id_num; i++) {
      if (id_num >= 100) { printf("yikes!\n"); exit(1); }
      if (idlist[i][0] == id) {
        idlist[i][1]++; 
        id_found = 1;
      }
    }
    if (id_found == 0) {
      idlist[id_num][0] = id;
      idlist[id_num][1] = 1;
      id_num++;
    }
  }

  mergewrite(fou4); // clean up adc, tdc hit lists

  printf("numEvts = %d\n", numEvts);
  for (i = 0; i < id_num; i++) { printf("evt_id = 0x%x, cnt = %d\n", idlist[i][0], idlist[i][1]); }
  
}
