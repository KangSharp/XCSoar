// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "FlarmNetDatabase.hpp"
#include "util/StringUtil.hpp"

#include <cassert>

void
FlarmNetDatabase::Insert(const FlarmNetRecord &record) noexcept
{
  FlarmId id = record.GetId();
  if (!id.IsDefined())
    /* ignore malformed records */
    return;

  map.insert(std::make_pair(id, record));
}

const FlarmNetRecord *
FlarmNetDatabase::FindFirstRecordByCallSign(const TCHAR *cn) const noexcept
{
  for (const auto &i : map) {
    assert(i.first.IsDefined());

    const FlarmNetRecord &record = i.second;
    if (StringIsEqual(record.callsign, cn))
      return &record;
  }

  return NULL;
}

unsigned
FlarmNetDatabase::FindRecordsByCallSign(const TCHAR *cn,
                                        const FlarmNetRecord *array[],
                                        [[maybe_unused]] unsigned size) const noexcept
{
  unsigned count = 0;

  for (const auto &i : map) {
    assert(i.first.IsDefined());

    const FlarmNetRecord &record = i.second;
    if (StringIsEqual(record.callsign, cn))
      array[count++] = &record;
  }

  return count;
}

unsigned
FlarmNetDatabase::FindIdsByCallSign(const TCHAR *cn, FlarmId array[],
                                    [[maybe_unused]] unsigned size) const noexcept
{
  unsigned count = 0;

  for (const auto &i : map) {
    assert(i.first.IsDefined());

    const FlarmNetRecord &record = i.second;
    if (StringIsEqual(record.callsign, cn))
      array[count++] = i.first;
  }

  return count;
}
