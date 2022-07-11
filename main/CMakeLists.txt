# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCS "main.c"
					"../main/simcom7070/simcom7070.c"
					"../main/string_user/location_parser.c"
					"../main/string_user/string_parse.c"
					"../main/simcom7070/gps_nmea/GPS.c")
set(COMPONENT_ADD_INCLUDEDIRS "")

register_component()
