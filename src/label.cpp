#include <routingkit/label.h>

Label::Label() : label(0) {}

Label::Label(uint64_t label) : label(label) {}

Label Label::fully_restricted() {
    Label l = Label();
    l.invert();
    return l;
}

uint64_t Label::get_label() const {
    return label;
}

void Label::set_label(uint64_t new_label) {
    label = new_label;
}

bool Label::operator==(const Label& other) const {
    return label == other.label;
}

bool Label::operator!=(const Label& other) const {
    return label != other.label;
}

/* 
* Checks if the label is allowed under the given restriction.
* The label is allowed if the disjunction of the labels and the restrictions is empty.
*/
bool Label::is_allowed(Label& restriction) const {
    return (label & restriction.label) == 0;
}

void Label::set_bit(bool bit, unsigned bit_position) {
    if (bit_position >= 64) {
        throw std::out_of_range("Bit position must be between 0 and 63");
    }
    if (bit) {
        label |= (1ULL << bit_position);
    } else {
        label &= ~(1ULL << bit_position);
    }
}

bool Label::get_bit(unsigned bit_position) const {
    if (bit_position >= 64) {
        throw std::out_of_range("Bit position must be between 0 and 63");
    }
    return (label & (1ULL << bit_position)) != 0;
}

bool Label::is_subset_of(const Label& other) const {
    return (label & other.label) == label;
}

bool Label::is_superset_of(const Label& other) const {
    return other.is_subset_of(*this);
}

Label Label::unite(const Label& other) const {
    return Label(label | other.label);
}
Label Label::intersect(const Label& other) const {
    return Label(label & other.label);
}

void Label::invert() {
    label = ~label;
}
