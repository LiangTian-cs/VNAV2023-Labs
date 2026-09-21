#include "random_vector.h"

#include <cstdlib>
#include <stdexcept>

RandomVector::RandomVector(int size, double max_val) {
  if (size <= 0) {
    throw std::invalid_argument("size must be greater than zero");
  }

  if (max_val < 0.0) {
    throw std::invalid_argument("max_val must be non-negative");
  }

  for (int i = 0; i < size; ++i) {
    double value =
        static_cast<double>(std::rand()) /
        static_cast<double>(RAND_MAX);

    vect.push_back(value * max_val);
  }
}

void RandomVector::print() {
  for (std::size_t i = 0; i < vect.size(); ++i) {
    std::cout << vect[i];

    if (i + 1 < vect.size()) {
      std::cout << " ";
    }
  }

  std::cout << std::endl;
}

double RandomVector::mean() {
  double sum = 0.0;

  for (std::size_t i = 0; i < vect.size(); ++i) {
    sum += vect[i];
  }

  return sum / static_cast<double>(vect.size());
}

double RandomVector::max() {
  double maximum = vect[0];

  for (std::size_t i = 1; i < vect.size(); ++i) {
    if (vect[i] > maximum) {
      maximum = vect[i];
    }
  }

  return maximum;
}

double RandomVector::min() {
  double minimum = vect[0];

  for (std::size_t i = 1; i < vect.size(); ++i) {
    if (vect[i] < minimum) {
      minimum = vect[i];
    }
  }

  return minimum;
}

void RandomVector::printHistogram(int bins) {
  if (bins <= 0) {
    throw std::invalid_argument("bins must be greater than zero");
  }

  double minimum = min();
  double maximum = max();

  std::vector<int> histogram(bins, 0);

  if (minimum == maximum) {
    histogram[0] = static_cast<int>(vect.size());
  } else {
    double bin_width =
        (maximum - minimum) / static_cast<double>(bins);

    for (std::size_t i = 0; i < vect.size(); ++i) {
      int bin =
          static_cast<int>((vect[i] - minimum) / bin_width);

      // max() lies exactly on the upper edge, so place it
      // in the final bin.
      if (bin >= bins) {
        bin = bins - 1;
      }

      ++histogram[bin];
    }
  }

  int max_height = histogram[0];

  for (int i = 1; i < bins; ++i) {
    if (histogram[i] > max_height) {
      max_height = histogram[i];
    }
  }

  for (int height = max_height; height > 0; --height) {
    for (int i = 0; i < bins; ++i) {
      if (histogram[i] >= height) {
        std::cout << "***";
      } else {
        std::cout << "   ";
      }

      if (i != bins - 1) {
        std::cout << " ";
      }
    }

    std::cout << std::endl;
  }
}
