#define LAT_XTEXT(X) #X
#define LAT_TEXT(X) LAT_XTEXT(X)

#define LAT_VERSION_MAJOR              1
#define LAT_VERSION_MINOR              0
#define LAT_VERSION_REVISION           0
#define LAT_VERSION_BUILD              0

#define LAT_VER_FILE_DESCRIPTION_STR   "LIDAR's algorithms test laboratory"
#define LAT_VER_FILE_VERSION           LAT_VERSION_MAJOR, LAT_VERSION_MINOR, LAT_VERSION_REVISION, LAT_VERSION_BUILD
#define LAT_VER_FILE_VERSION_STR       LAT_TEXT(VERSION_MAJOR)    \
										"." LAT_TEXT(VERSION_MINOR)    \
										"." LAT_TEXT(VERSION_REVISION) \
										"." LAT_TEXT(VERSION_BUILD)    \

#define LAT_VER_COMPANY_NAME_STR	    "www.robotics.by and/or its subsidiary(-ies)"
#define LAT_VER_PRODUCTNAME_STR         "LIDAR's algorithms test laboratory"
#define LAT_VER_PRODUCT_VERSION         LAT_VER_FILE_VERSION
#define LAT_VER_PRODUCT_VERSION_STR     LAT_VER_FILE_VERSION_STR
#define LAT_VER_ORIGINAL_FILENAME_STR   LAT_VER_PRODUCTNAME_STR ".exe"
#define LAT_VER_INTERNAL_NAME_STR       "LIDAR's algorithms test laboratory"
#define LAT_VER_COPYRIGHT_STR           "Copyright (C) 2024 Siarhei Herasiuta <contacts@robotics.by> and/or its subsidiary(-ies)."

#define LAT_RELEASE "Compiled " __DATE__ " " __TIME__

static const char *licenseLATLaboratory = 
	"Copyright (C) 2024 Siarhei Herasiuta <contacts@robotics.by>\n"
	"\n"
	"http://www.robotics.by/\n"
	"\n"
	"All rights reserved.\n"
	"\n"
	"Redistribution and use in source and binary forms, with or without\n"
	"modification, are permitted provided that the following conditions\n"
	"are met:\n"
	"\n"
	"- Redistributions of source code must retain the above copyright notice,\n"
	"  this list of conditions and the following disclaimer.\n"
	"- Redistributions in binary form must reproduce the above copyright notice,\n"
	"  this list of conditions and the following disclaimer in the documentation\n"
	"  and/or other materials provided with the distribution.\n"
	"- Neither the name of the BelAZ Control server Developers nor the names of its\n"
	"  contributors may be used to endorse or promote products derived from this\n"
	"  software without specific prior written permission.\n"
	"\n"
	"THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS\n"
	"`AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT\n"
	"LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR\n"
	"A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR\n"
	"CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,\n"
	"EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,\n"
	"PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR\n"
	"PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF\n"
	"LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING\n"
	"NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS\n"
	"SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.";