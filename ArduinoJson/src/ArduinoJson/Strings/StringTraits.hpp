// ArduinoJson - arduinojson.org
// Copyright Benoit Blanchon 2014-2018
// MIT License

#pragma once

#include <string.h>
#include "../Configuration.hpp"
#include "../Polyfills/type_traits.hpp"

namespace ArduinoJson {
namespace Internals {

template <typename TString, typename Enable = void>
struct StringTraits {
  static const bool has_append = false;
  static const bool has_equals = false;
};

template <typename TString>
struct StringTraits<const TString, void> : StringTraits<TString> {};

template <typename TString>
struct StringTraits<TString&, void> : StringTraits<TString> {};
}  // namespace Internals
}  // namespace ArduinoJson

#include "CharPointer.hpp"
#include "FlashString.hpp"
#include "StdString.hpp"
