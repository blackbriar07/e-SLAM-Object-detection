#include "NuSTM32_C++.h"
#include "NuSTM32_JSON.h"

int strFind(char szText[], const char* sz)
{
	char* szClose = strstr(szText, sz);
	if (!szClose)
		return -1;
	return strlen(szText) - strlen(szClose);
}

void strtrim(char szText[])
{
	int nLen = strlen(szText);
	char szRight[MAX_JSON_VALUE];
	memset(szRight, 0, MAX_JSON_VALUE);
	int i;
	for (i = 0; i < nLen; i++)
	{
		if (szText[i] != ' ')
		{
			memcpy(szRight, &szText[i], MAX_JSON_VALUE - i);
			memcpy(szText, szRight, MAX_JSON_VALUE);	
			break;
		}
	}
	nLen = strlen(szText);
	memset(szRight, 0, MAX_JSON_VALUE);
	for (i = nLen - 1; i >= 0; i--)
	{
		if (szText[i] != ' ')
		{
			memcpy(szRight, szText, i + 1);
			break;
		}
	}
	memcpy(szText, szRight, MAX_JSON_VALUE);	
}

void strsplit(char szText[], char szParam[], const char* szSymbol)
{
	char szRight[MAX_JSON_VALUE];
	memset(szRight, 0, MAX_JSON_VALUE);
	memset(szParam, 0, MAX_JSON_VALUE);
	int nPos = strFind(szText, szSymbol);
	if (nPos < 0)
	{
		memcpy(szText, szParam, MAX_JSON_VALUE);
		return;
	}
	memcpy(szRight, &szText[nPos + 1], MAX_JSON_VALUE - (nPos + 1));
	if (nPos)
		memcpy(szParam, szText, nPos);
	memcpy(szText, szRight, MAX_JSON_VALUE);
	strtrim(szParam);
}

int GCD(int A, int B)
{
	if (B)
		return GCD(B, A % B);
	else
		return A;
}

double _fabs(double a)
{
	return a > 0? a: -a;
}

double _fmax(double x, double y)
{
	return x > y? x: y;
}

double _fmin(double x, double y)
{
	return x < y? x: y;
}

int _iabs(int a)
{
	return a > 0? a: -a;
}

int _imax(int x, int y)
{
	return x > y? x: y;
}

int _imin(int x, int y)
{
	return x < y? x: y;
}

int Bound(int imax, int imin, int ivalue)
{
	return _imin(imax, _imax(imin, ivalue));
}

////////////////////////////////////////

uint32_t sineTable[SINETABLE_SIZE];

uint16_t sinegenerator(double x)
{
	double sinx = 0;
	double dSum = x;
	double x2 = x * x;
	for (int i = 1; i < 10; i += 2)
	{
		sinx += dSum;
		dSum *= -x2 / ((i + 2) * (i + 1));
	}
	return 0x800 * sinx + 0x7FF;
}

void CreateSineTable()
{
	for (int i = 0; i < SINETABLE_SIZE; i++)
	{
		if (i <= SINETABLE_SIZE / 4)
			sineTable[i] = sinegenerator((double) PI2 * i / SINETABLE_SIZE);
		else if (i > SINETABLE_SIZE / 2)
			sineTable[i] = 0xFFD - sineTable[SINETABLE_SIZE - i];
		else
			sineTable[i] = sineTable[SINETABLE_SIZE / 2 - i];
	}
}

double _cos(double x)
{
	return sineTable[(int)(x / PI2 * SINETABLE_SIZE + SINETABLE_SIZE / 4)  % SINETABLE_SIZE];
}
double _sin(double x)
{
	return sineTable[(int)(x / PI2 * SINETABLE_SIZE)  % SINETABLE_SIZE];
}
