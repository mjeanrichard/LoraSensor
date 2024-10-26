#pragma once

#define RETURN_ON_ERROR(x)         \
    do                                   \
    {                                    \
        esp_err_t err_rc_ = (x);         \
        if (unlikely(err_rc_ != ESP_OK)) \
        {                                \
            return err_rc_;              \
        }                                \
    } while (0)
