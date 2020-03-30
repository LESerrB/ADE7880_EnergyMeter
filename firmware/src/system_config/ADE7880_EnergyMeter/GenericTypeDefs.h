/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef _EXAMPLE_FILE_NAME_H    /* Guard against multiple inclusion */
#define _EXAMPLE_FILE_NAME_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */

/* TODO:  Include other files here if needed. */


/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif


    /* ************************************************************************** */
    /* ************************************************************************** */
    /* Section: Constants                                                         */
    /* ************************************************************************** */
    /* ************************************************************************** */

    /*  A brief description of a section can be given directly below the section
        banner.
     */


    /* ************************************************************************** */
    /** Descriptive Constant Name

      @Summary
        Brief one-line summary of the constant.
    
      @Description
        Full description, explaining the purpose and usage of the constant.
        <p>
        Additional description in consecutive paragraphs separated by HTML 
        paragraph breaks, as necessary.
        <p>
        Type "JavaDoc" in the "How Do I?" IDE toolbar for more information on tags.
    
      @Remarks
        Any additional remarks
     */
//#define EXAMPLE_CONSTANT 0
    
#define __EXTENSION
    
#define __PACKED


    // *****************************************************************************
    // *****************************************************************************
    // Section: Data Types
    // *****************************************************************************
    // *****************************************************************************

    /*  A brief description of a section can be given directly below the section
        banner.
     */


    // *****************************************************************************

    /* Alternate definitions */
    
typedef enum _BOOL { FALSE = 0, TRUE } BOOL;    /* Undefined size */
typedef enum _BIT { CLEAR = 0, SET } BIT;

typedef void                    VOID;

typedef char                    CHAR8;
typedef unsigned char           UCHAR8;

typedef unsigned char           BYTE;                           /* 8-bit unsigned  */
typedef unsigned short int      WORD;                           /* 16-bit unsigned */
typedef unsigned long           DWORD;                          /* 32-bit unsigned */
/* MPLAB C Compiler for PIC18 does not support 64-bit integers */
__EXTENSION
typedef unsigned long long      QWORD;                          /* 64-bit unsigned */
typedef signed char             CHAR;                           /* 8-bit signed    */
typedef signed short int        SHORT;                          /* 16-bit signed   */
typedef signed long             LONG;                           /* 32-bit signed   */
/* MPLAB C Compiler for PIC18 does not support 64-bit integers */
__EXTENSION
typedef signed long long        LONGLONG;                       /* 64-bit signed   */
typedef union
{
    BYTE Val;
    struct __PACKED
    {
        __EXTENSION BYTE b0:1;
        __EXTENSION BYTE b1:1;
        __EXTENSION BYTE b2:1;
        __EXTENSION BYTE b3:1;
        __EXTENSION BYTE b4:1;
        __EXTENSION BYTE b5:1;
        __EXTENSION BYTE b6:1;
        __EXTENSION BYTE b7:1;
    } bits;
} BYTE_VAL, BYTE_BITS;

typedef union
{
    WORD Val;
    BYTE v[2] __PACKED;
    struct __PACKED
    {
        BYTE LB;
        BYTE HB;
    } byte;
    struct __PACKED
    {
        __EXTENSION BYTE b0:1;
        __EXTENSION BYTE b1:1;
        __EXTENSION BYTE b2:1;
        __EXTENSION BYTE b3:1;
        __EXTENSION BYTE b4:1;
        __EXTENSION BYTE b5:1;
        __EXTENSION BYTE b6:1;
        __EXTENSION BYTE b7:1;
        __EXTENSION BYTE b8:1;
        __EXTENSION BYTE b9:1;
        __EXTENSION BYTE b10:1;
        __EXTENSION BYTE b11:1;
        __EXTENSION BYTE b12:1;
        __EXTENSION BYTE b13:1;
        __EXTENSION BYTE b14:1;
        __EXTENSION BYTE b15:1;
    } bits;
} WORD_VAL, WORD_BITS;

typedef union
{
    DWORD Val;
    WORD w[2] __PACKED;
    BYTE v[4] __PACKED;
    struct __PACKED
    {
        WORD LW;
        WORD HW;
    } word;
    struct __PACKED
    {
        BYTE LB;
        BYTE HB;
        BYTE UB;
        BYTE MB;
    } byte;
    struct __PACKED
    {
        WORD_VAL low;
        WORD_VAL high;
    }wordUnion;
    struct __PACKED
    {
        __EXTENSION BYTE b0:1;
        __EXTENSION BYTE b1:1;
        __EXTENSION BYTE b2:1;
        __EXTENSION BYTE b3:1;
        __EXTENSION BYTE b4:1;
        __EXTENSION BYTE b5:1;
        __EXTENSION BYTE b6:1;
        __EXTENSION BYTE b7:1;
        __EXTENSION BYTE b8:1;
        __EXTENSION BYTE b9:1;
        __EXTENSION BYTE b10:1;
        __EXTENSION BYTE b11:1;
        __EXTENSION BYTE b12:1;
        __EXTENSION BYTE b13:1;
        __EXTENSION BYTE b14:1;
        __EXTENSION BYTE b15:1;
        __EXTENSION BYTE b16:1;
        __EXTENSION BYTE b17:1;
        __EXTENSION BYTE b18:1;
        __EXTENSION BYTE b19:1;
        __EXTENSION BYTE b20:1;
        __EXTENSION BYTE b21:1;
        __EXTENSION BYTE b22:1;
        __EXTENSION BYTE b23:1;
        __EXTENSION BYTE b24:1;
        __EXTENSION BYTE b25:1;
        __EXTENSION BYTE b26:1;
        __EXTENSION BYTE b27:1;
        __EXTENSION BYTE b28:1;
        __EXTENSION BYTE b29:1;
        __EXTENSION BYTE b30:1;
        __EXTENSION BYTE b31:1;
    } bits;
} DWORD_VAL;

/* MPLAB C Compiler for PIC18 does not support 64-bit integers */
typedef union
{
    QWORD Val;
    DWORD d[2] __PACKED;
    WORD w[4] __PACKED;
    BYTE v[8] __PACKED;
    struct __PACKED
    {
        DWORD LD;
        DWORD HD;
    } dword;
    struct __PACKED
    {
        WORD LW;
        WORD HW;
        WORD UW;
        WORD MW;
    } word;
    struct __PACKED
    {
        __EXTENSION BYTE b0:1;
        __EXTENSION BYTE b1:1;
        __EXTENSION BYTE b2:1;
        __EXTENSION BYTE b3:1;
        __EXTENSION BYTE b4:1;
        __EXTENSION BYTE b5:1;
        __EXTENSION BYTE b6:1;
        __EXTENSION BYTE b7:1;
        __EXTENSION BYTE b8:1;
        __EXTENSION BYTE b9:1;
        __EXTENSION BYTE b10:1;
        __EXTENSION BYTE b11:1;
        __EXTENSION BYTE b12:1;
        __EXTENSION BYTE b13:1;
        __EXTENSION BYTE b14:1;
        __EXTENSION BYTE b15:1;
        __EXTENSION BYTE b16:1;
        __EXTENSION BYTE b17:1;
        __EXTENSION BYTE b18:1;
        __EXTENSION BYTE b19:1;
        __EXTENSION BYTE b20:1;
        __EXTENSION BYTE b21:1;
        __EXTENSION BYTE b22:1;
        __EXTENSION BYTE b23:1;
        __EXTENSION BYTE b24:1;
        __EXTENSION BYTE b25:1;
        __EXTENSION BYTE b26:1;
        __EXTENSION BYTE b27:1;
        __EXTENSION BYTE b28:1;
        __EXTENSION BYTE b29:1;
        __EXTENSION BYTE b30:1;
        __EXTENSION BYTE b31:1;
        __EXTENSION BYTE b32:1;
        __EXTENSION BYTE b33:1;
        __EXTENSION BYTE b34:1;
        __EXTENSION BYTE b35:1;
        __EXTENSION BYTE b36:1;
        __EXTENSION BYTE b37:1;
        __EXTENSION BYTE b38:1;
        __EXTENSION BYTE b39:1;
        __EXTENSION BYTE b40:1;
        __EXTENSION BYTE b41:1;
        __EXTENSION BYTE b42:1;
        __EXTENSION BYTE b43:1;
        __EXTENSION BYTE b44:1;
        __EXTENSION BYTE b45:1;
        __EXTENSION BYTE b46:1;
        __EXTENSION BYTE b47:1;
        __EXTENSION BYTE b48:1;
        __EXTENSION BYTE b49:1;
        __EXTENSION BYTE b50:1;
        __EXTENSION BYTE b51:1;
        __EXTENSION BYTE b52:1;
        __EXTENSION BYTE b53:1;
        __EXTENSION BYTE b54:1;
        __EXTENSION BYTE b55:1;
        __EXTENSION BYTE b56:1;
        __EXTENSION BYTE b57:1;
        __EXTENSION BYTE b58:1;
        __EXTENSION BYTE b59:1;
        __EXTENSION BYTE b60:1;
        __EXTENSION BYTE b61:1;
        __EXTENSION BYTE b62:1;
        __EXTENSION BYTE b63:1;
    } bits;
} QWORD_VAL;
    

typedef union
{
	struct
	{
		unsigned char	rsvd;		// reserved for future use. should be 0
		unsigned char	sec;		// BCD codification for seconds, 00-59
		unsigned char	min;		// BCD codification for minutes, 00-59
		unsigned char	hour;		// BCD codification for hours, 00-24
	};								// field access
	unsigned char		b[4];		// byte access
	unsigned short		w[2];		// 16 bits access
	unsigned long		l;			// 32 bits access
}rtccTime;


typedef union
{
	struct
	{
		unsigned char	wday;		// BCD codification for day of the week, 00-06
		unsigned char	mday;		// BCD codification for day of the month, 01-31
		unsigned char	mon;		// BCD codification for month, 01-12
		unsigned char	year;		// BCD codification for years, 00-99
	};								// field access
	unsigned char		b[4];		// byte access
	unsigned short		w[2];		// 16 bits access
	unsigned long		l;			// 32 bits access
}rtccDate;
    

    /** Descriptive Data Type Name

      @Summary
        Brief one-line summary of the data type.
    
      @Description
        Full description, explaining the purpose and usage of the data type.
        <p>
        Additional description in consecutive paragraphs separated by HTML 
        paragraph breaks, as necessary.
        <p>
        Type "JavaDoc" in the "How Do I?" IDE toolbar for more information on tags.

      @Remarks
        Any additional remarks
        <p>
        Describe enumeration elements and structure and union members above each 
        element or member.
     */
//    typedef struct _example_struct_t {
//        /* Describe structure member. */
//        int some_number;
//
//        /* Describe structure member. */
//        bool some_flag;
//
//    } example_struct_t;


    // *****************************************************************************
    // *****************************************************************************
    // Section: Interface Functions
    // *****************************************************************************
    // *****************************************************************************

    /*  A brief description of a section can be given directly below the section
        banner.
     */

    // *****************************************************************************
    /**
      @Function
        int ExampleFunctionName ( int param1, int param2 ) 

      @Summary
        Brief one-line description of the function.

      @Description
        Full description, explaining the purpose and usage of the function.
        <p>
        Additional description in consecutive paragraphs separated by HTML 
        paragraph breaks, as necessary.
        <p>
        Type "JavaDoc" in the "How Do I?" IDE toolbar for more information on tags.

      @Precondition
        List and describe any required preconditions. If there are no preconditions,
        enter "None."

      @Parameters
        @param param1 Describe the first parameter to the function.
    
        @param param2 Describe the second parameter to the function.

      @Returns
        List (if feasible) and describe the return values of the function.
        <ul>
          <li>1   Indicates an error occurred
          <li>0   Indicates an error did not occur
        </ul>

      @Remarks
        Describe any special behavior not described above.
        <p>
        Any additional remarks.

      @Example
        @code
        if(ExampleFunctionName(1, 2) == 0)
        {
            return 3;
        }
     */
    //int ExampleFunction(int param1, int param2);


    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
