  0: char <= pc[31:28];
  1: char <= pc[27:24];
  2: char <= pc[23:20];
  3: char <= pc[19:16];
  4: char <= pc[15:12];
  5: char <= pc[11:8];
  6: char <= pc[7:4];
  7: char <= pc[3:0];
  8: char <= 8'h20+8'h80;
  9: char <= 8'h5b+8'h80;
 10: char <= ireg[31:28];
 11: char <= ireg[27:24];
 12: char <= ireg[23:20];
 13: char <= ireg[19:16];
 14: char <= ireg[15:12];
 15: char <= ireg[11:8];
 16: char <= ireg[7:4];
 17: char <= ireg[3:0];
 18: char <= 8'h5d+8'h80;
 19: char <= 8'h20+8'h80;
 20: char <= 8'h0;
 21: char <= 8'h3a+8'h80;
 22: char <= xreg[0][31:28];
 23: char <= xreg[0][27:24];
 24: char <= xreg[0][23:20];
 25: char <= xreg[0][19:16];
 26: char <= xreg[0][15:12];
 27: char <= xreg[0][11:8];
 28: char <= xreg[0][7:4];
 29: char <= xreg[0][3:0];
 30: char <= 8'h20+8'h80;
 31: char <= 8'h1;
 32: char <= 8'h3a+8'h80;
 33: char <= xreg[1][31:28];
 34: char <= xreg[1][27:24];
 35: char <= xreg[1][23:20];
 36: char <= xreg[1][19:16];
 37: char <= xreg[1][15:12];
 38: char <= xreg[1][11:8];
 39: char <= xreg[1][7:4];
 40: char <= xreg[1][3:0];
 41: char <= 8'h20+8'h80;
 42: char <= 8'h2;
 43: char <= 8'h3a+8'h80;
 44: char <= xreg[2][31:28];
 45: char <= xreg[2][27:24];
 46: char <= xreg[2][23:20];
 47: char <= xreg[2][19:16];
 48: char <= xreg[2][15:12];
 49: char <= xreg[2][11:8];
 50: char <= xreg[2][7:4];
 51: char <= xreg[2][3:0];
 52: char <= 8'h20+8'h80;
 53: char <= 8'h3;
 54: char <= 8'h3a+8'h80;
 55: char <= xreg[3][31:28];
 56: char <= xreg[3][27:24];
 57: char <= xreg[3][23:20];
 58: char <= xreg[3][19:16];
 59: char <= xreg[3][15:12];
 60: char <= xreg[3][11:8];
 61: char <= xreg[3][7:4];
 62: char <= xreg[3][3:0];
 63: char <= 8'h20+8'h80;
 64: char <= 8'h4;
 65: char <= 8'h3a+8'h80;
 66: char <= xreg[4][31:28];
 67: char <= xreg[4][27:24];
 68: char <= xreg[4][23:20];
 69: char <= xreg[4][19:16];
 70: char <= xreg[4][15:12];
 71: char <= xreg[4][11:8];
 72: char <= xreg[4][7:4];
 73: char <= xreg[4][3:0];
 74: char <= 8'h20+8'h80;
 75: char <= 8'h5;
 76: char <= 8'h3a+8'h80;
 77: char <= xreg[5][31:28];
 78: char <= xreg[5][27:24];
 79: char <= xreg[5][23:20];
 80: char <= xreg[5][19:16];
 81: char <= xreg[5][15:12];
 82: char <= xreg[5][11:8];
 83: char <= xreg[5][7:4];
 84: char <= xreg[5][3:0];
 85: char <= 8'h20+8'h80;
 86: char <= 8'h6;
 87: char <= 8'h3a+8'h80;
 88: char <= xreg[6][31:28];
 89: char <= xreg[6][27:24];
 90: char <= xreg[6][23:20];
 91: char <= xreg[6][19:16];
 92: char <= xreg[6][15:12];
 93: char <= xreg[6][11:8];
 94: char <= xreg[6][7:4];
 95: char <= xreg[6][3:0];
 96: char <= 8'h20+8'h80;
 97: char <= 8'h7;
 98: char <= 8'h3a+8'h80;
 99: char <= xreg[7][31:28];
100: char <= xreg[7][27:24];
101: char <= xreg[7][23:20];
102: char <= xreg[7][19:16];
103: char <= xreg[7][15:12];
104: char <= xreg[7][11:8];
105: char <= xreg[7][7:4];
106: char <= xreg[7][3:0];
107: char <= 8'h20+8'h80;
108: char <= 8'h8;
109: char <= 8'h3a+8'h80;
110: char <= xreg[8][31:28];
111: char <= xreg[8][27:24];
112: char <= xreg[8][23:20];
113: char <= xreg[8][19:16];
114: char <= xreg[8][15:12];
115: char <= xreg[8][11:8];
116: char <= xreg[8][7:4];
117: char <= xreg[8][3:0];
118: char <= 8'h20+8'h80;
119: char <= 8'h9;
120: char <= 8'h3a+8'h80;
121: char <= xreg[9][31:28];
122: char <= xreg[9][27:24];
123: char <= xreg[9][23:20];
124: char <= xreg[9][19:16];
125: char <= xreg[9][15:12];
126: char <= xreg[9][11:8];
127: char <= xreg[9][7:4];
128: char <= xreg[9][3:0];
129: char <= 8'h20+8'h80;
130: char <= 8'h1;
131: char <= 8'h0;
132: char <= 8'h3a+8'h80;
133: char <= xreg[10][31:28];
134: char <= xreg[10][27:24];
135: char <= xreg[10][23:20];
136: char <= xreg[10][19:16];
137: char <= xreg[10][15:12];
138: char <= xreg[10][11:8];
139: char <= xreg[10][7:4];
140: char <= xreg[10][3:0];
141: char <= 8'h20+8'h80;
142: char <= 8'h1;
143: char <= 8'h1;
144: char <= 8'h3a+8'h80;
145: char <= xreg[11][31:28];
146: char <= xreg[11][27:24];
147: char <= xreg[11][23:20];
148: char <= xreg[11][19:16];
149: char <= xreg[11][15:12];
150: char <= xreg[11][11:8];
151: char <= xreg[11][7:4];
152: char <= xreg[11][3:0];
153: char <= 8'h20+8'h80;
154: char <= 8'h1;
155: char <= 8'h2;
156: char <= 8'h3a+8'h80;
157: char <= xreg[12][31:28];
158: char <= xreg[12][27:24];
159: char <= xreg[12][23:20];
160: char <= xreg[12][19:16];
161: char <= xreg[12][15:12];
162: char <= xreg[12][11:8];
163: char <= xreg[12][7:4];
164: char <= xreg[12][3:0];
165: char <= 8'h20+8'h80;
166: char <= 8'h1;
167: char <= 8'h3;
168: char <= 8'h3a+8'h80;
169: char <= xreg[13][31:28];
170: char <= xreg[13][27:24];
171: char <= xreg[13][23:20];
172: char <= xreg[13][19:16];
173: char <= xreg[13][15:12];
174: char <= xreg[13][11:8];
175: char <= xreg[13][7:4];
176: char <= xreg[13][3:0];
177: char <= 8'h20+8'h80;
178: char <= 8'h1;
179: char <= 8'h4;
180: char <= 8'h3a+8'h80;
181: char <= xreg[14][31:28];
182: char <= xreg[14][27:24];
183: char <= xreg[14][23:20];
184: char <= xreg[14][19:16];
185: char <= xreg[14][15:12];
186: char <= xreg[14][11:8];
187: char <= xreg[14][7:4];
188: char <= xreg[14][3:0];
189: char <= 8'h20+8'h80;
190: char <= 8'h1;
191: char <= 8'h5;
192: char <= 8'h3a+8'h80;
193: char <= xreg[15][31:28];
194: char <= xreg[15][27:24];
195: char <= xreg[15][23:20];
196: char <= xreg[15][19:16];
197: char <= xreg[15][15:12];
198: char <= xreg[15][11:8];
199: char <= xreg[15][7:4];
200: char <= xreg[15][3:0];
201: char <= 8'h20+8'h80;
202: char <= 8'h1;
203: char <= 8'h6;
204: char <= 8'h3a+8'h80;
205: char <= xreg[16][31:28];
206: char <= xreg[16][27:24];
207: char <= xreg[16][23:20];
208: char <= xreg[16][19:16];
209: char <= xreg[16][15:12];
210: char <= xreg[16][11:8];
211: char <= xreg[16][7:4];
212: char <= xreg[16][3:0];
213: char <= 8'h20+8'h80;
214: char <= 8'h1;
215: char <= 8'h7;
216: char <= 8'h3a+8'h80;
217: char <= xreg[17][31:28];
218: char <= xreg[17][27:24];
219: char <= xreg[17][23:20];
220: char <= xreg[17][19:16];
221: char <= xreg[17][15:12];
222: char <= xreg[17][11:8];
223: char <= xreg[17][7:4];
224: char <= xreg[17][3:0];
225: char <= 8'h20+8'h80;
226: char <= 8'h1;
227: char <= 8'h8;
228: char <= 8'h3a+8'h80;
229: char <= xreg[18][31:28];
230: char <= xreg[18][27:24];
231: char <= xreg[18][23:20];
232: char <= xreg[18][19:16];
233: char <= xreg[18][15:12];
234: char <= xreg[18][11:8];
235: char <= xreg[18][7:4];
236: char <= xreg[18][3:0];
237: char <= 8'h20+8'h80;
238: char <= 8'h1;
239: char <= 8'h9;
240: char <= 8'h3a+8'h80;
241: char <= xreg[19][31:28];
242: char <= xreg[19][27:24];
243: char <= xreg[19][23:20];
244: char <= xreg[19][19:16];
245: char <= xreg[19][15:12];
246: char <= xreg[19][11:8];
247: char <= xreg[19][7:4];
248: char <= xreg[19][3:0];
249: char <= 8'h20+8'h80;
250: char <= 8'h2;
251: char <= 8'h0;
252: char <= 8'h3a+8'h80;
253: char <= xreg[20][31:28];
254: char <= xreg[20][27:24];
255: char <= xreg[20][23:20];
256: char <= xreg[20][19:16];
257: char <= xreg[20][15:12];
258: char <= xreg[20][11:8];
259: char <= xreg[20][7:4];
260: char <= xreg[20][3:0];
261: char <= 8'h20+8'h80;
262: char <= 8'h2;
263: char <= 8'h1;
264: char <= 8'h3a+8'h80;
265: char <= xreg[21][31:28];
266: char <= xreg[21][27:24];
267: char <= xreg[21][23:20];
268: char <= xreg[21][19:16];
269: char <= xreg[21][15:12];
270: char <= xreg[21][11:8];
271: char <= xreg[21][7:4];
272: char <= xreg[21][3:0];
273: char <= 8'h20+8'h80;
274: char <= 8'h2;
275: char <= 8'h2;
276: char <= 8'h3a+8'h80;
277: char <= xreg[22][31:28];
278: char <= xreg[22][27:24];
279: char <= xreg[22][23:20];
280: char <= xreg[22][19:16];
281: char <= xreg[22][15:12];
282: char <= xreg[22][11:8];
283: char <= xreg[22][7:4];
284: char <= xreg[22][3:0];
285: char <= 8'h20+8'h80;
286: char <= 8'h2;
287: char <= 8'h3;
288: char <= 8'h3a+8'h80;
289: char <= xreg[23][31:28];
290: char <= xreg[23][27:24];
291: char <= xreg[23][23:20];
292: char <= xreg[23][19:16];
293: char <= xreg[23][15:12];
294: char <= xreg[23][11:8];
295: char <= xreg[23][7:4];
296: char <= xreg[23][3:0];
297: char <= 8'h20+8'h80;
298: char <= 8'h2;
299: char <= 8'h4;
300: char <= 8'h3a+8'h80;
301: char <= xreg[24][31:28];
302: char <= xreg[24][27:24];
303: char <= xreg[24][23:20];
304: char <= xreg[24][19:16];
305: char <= xreg[24][15:12];
306: char <= xreg[24][11:8];
307: char <= xreg[24][7:4];
308: char <= xreg[24][3:0];
309: char <= 8'h20+8'h80;
310: char <= 8'h2;
311: char <= 8'h5;
312: char <= 8'h3a+8'h80;
313: char <= xreg[25][31:28];
314: char <= xreg[25][27:24];
315: char <= xreg[25][23:20];
316: char <= xreg[25][19:16];
317: char <= xreg[25][15:12];
318: char <= xreg[25][11:8];
319: char <= xreg[25][7:4];
320: char <= xreg[25][3:0];
321: char <= 8'h20+8'h80;
322: char <= 8'h2;
323: char <= 8'h6;
324: char <= 8'h3a+8'h80;
325: char <= xreg[26][31:28];
326: char <= xreg[26][27:24];
327: char <= xreg[26][23:20];
328: char <= xreg[26][19:16];
329: char <= xreg[26][15:12];
330: char <= xreg[26][11:8];
331: char <= xreg[26][7:4];
332: char <= xreg[26][3:0];
333: char <= 8'h20+8'h80;
334: char <= 8'h2;
335: char <= 8'h7;
336: char <= 8'h3a+8'h80;
337: char <= xreg[27][31:28];
338: char <= xreg[27][27:24];
339: char <= xreg[27][23:20];
340: char <= xreg[27][19:16];
341: char <= xreg[27][15:12];
342: char <= xreg[27][11:8];
343: char <= xreg[27][7:4];
344: char <= xreg[27][3:0];
345: char <= 8'h20+8'h80;
346: char <= 8'h2;
347: char <= 8'h8;
348: char <= 8'h3a+8'h80;
349: char <= xreg[28][31:28];
350: char <= xreg[28][27:24];
351: char <= xreg[28][23:20];
352: char <= xreg[28][19:16];
353: char <= xreg[28][15:12];
354: char <= xreg[28][11:8];
355: char <= xreg[28][7:4];
356: char <= xreg[28][3:0];
357: char <= 8'h20+8'h80;
358: char <= 8'h2;
359: char <= 8'h9;
360: char <= 8'h3a+8'h80;
361: char <= xreg[29][31:28];
362: char <= xreg[29][27:24];
363: char <= xreg[29][23:20];
364: char <= xreg[29][19:16];
365: char <= xreg[29][15:12];
366: char <= xreg[29][11:8];
367: char <= xreg[29][7:4];
368: char <= xreg[29][3:0];
369: char <= 8'h20+8'h80;
370: char <= 8'h3;
371: char <= 8'h0;
372: char <= 8'h3a+8'h80;
373: char <= xreg[30][31:28];
374: char <= xreg[30][27:24];
375: char <= xreg[30][23:20];
376: char <= xreg[30][19:16];
377: char <= xreg[30][15:12];
378: char <= xreg[30][11:8];
379: char <= xreg[30][7:4];
380: char <= xreg[30][3:0];
381: char <= 8'h20+8'h80;
382: char <= 8'h3;
383: char <= 8'h1;
384: char <= 8'h3a+8'h80;
385: char <= xreg[31][31:28];
386: char <= xreg[31][27:24];
387: char <= xreg[31][23:20];
388: char <= xreg[31][19:16];
389: char <= xreg[31][15:12];
390: char <= xreg[31][11:8];
391: char <= xreg[31][7:4];
392: char <= xreg[31][3:0];

//393: char <= 8'h0a+8'h80;    // '\n'

393: char <= 8'h20+8'h80;
394: char <= memval[31:28];
395: char <= memval[27:24];
396: char <= memval[23:20];
397: char <= memval[19:16];
398: char <= memval[15:12];
399: char <= memval[11:8];
400: char <= memval[7:4];
401: char <= memval[3:0];

/*393: char <= 8'h20+8'h80;
394: char <= timerl[31:28];
395: char <= timerl[27:24];
396: char <= timerl[23:20];
397: char <= timerl[19:16];
398: char <= timerl[15:12];
399: char <= timerl[11:8];
400: char <= timerl[7:4];
401: char <= timerl[3:0];*/

402: char <= 8'h0a+8'h80;    // '\n'

