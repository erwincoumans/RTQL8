INCLUDE_DIRECTORIES(
	.
)

ADD_DEFINITIONS(-DTIXML_USE_TICPP)

ADD_LIBRARY(ticpp STATIC
	ticpp.cpp
	tinystr.cpp
	tinyxml.cpp
	tinyxmlerror.cpp
	tinyxmlparser.cpp
	tinyxml.h
	tinystr.h
	ticpprc.h
	ticpp.h
)

