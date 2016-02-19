#include "atcommon.h"

/*
 * qXfer:object:read:annex:offset,length
 * qXfer:pin:read:d|a:(pinname)
 *
 * qXfer:object:write:annex:offset:data
 * qXfer:pin:write:d|a:(pinname):value
 *
 */

static int atcmn_pin_query(char *in_buf, char *out_buf, size_t out_buf_size)
{
    int analog;
    if (strncmp(in_buf, "read:", 5) == 0)
    {
        /* Access device pins */
        atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                            "%s: atcmn_pin_query(read)",
                            atuc3_target.name);

        if (strncmp(in_buf+6, "a:", 2) == 0)
            analog = TRUE;
        else if (strncmp(in_buf+6, "d:", 2) == 0)
            analog = FALSE;

        /* Read pin value */

        return RP_VAL_TARGETRET_NOSUPP;
    }
    if (strncmp(in_buf, "write:", 6) == 0)
    {
        /* Access device pins */
        atuc3_status.log(RP_VAL_LOGLEVEL_DEBUG,
                            "%s: atcmn_pin_query(write)",
                            atuc3_target.name);

        if (strncmp(in_buf+7, "a:", 2) == 0)
            analog = TRUE;
        else if (strncmp(in_buf+7, "d:", 2) == 0)
            analog = FALSE;

        /* Read pin value */

        return RP_VAL_TARGETRET_NOSUPP;
    }
    return RP_VAL_TARGETRET_NOSUPP;

}
