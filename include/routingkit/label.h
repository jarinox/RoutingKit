#ifndef LABEL_H
#define LABEL_H

#include <cstdint>
#include <stdexcept>

class Label {
private:
    uint64_t label;
public:
    Label();
    explicit Label(uint64_t label);
    uint64_t get_label() const;
    void set_label(uint64_t new_label);
    bool operator==(const Label& other) const;
    bool operator!=(const Label& other) const;
    bool is_allowed(uint64_t restriction) const;
    bool is_subset_of(const Label& other) const;
    bool is_superset_of(const Label& other) const;
    void set_bit(bool bit, unsigned bit_position);
    bool get_bit(unsigned bit_position) const;
};

#endif // LABEL_H
