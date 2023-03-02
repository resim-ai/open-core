#pragma once

namespace resim {

enum class HttpResponse {
  INVALID = -1,
  OK = 200,
  CREATED = 201,
  FORBIDDEN = 403,
  NOT_FOUND = 404,
};

}  // namespace resim
