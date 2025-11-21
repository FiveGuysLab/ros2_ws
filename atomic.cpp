#include <atomic>
#include <iostream>

int main() {
    std::cout << "Checking if std::atomic is lock-free for various types:\n\n";
    
    // Check using instance method
    std::cout << "Using std::atomic<T>::is_lock_free():\n";
    std::atomic<bool> atomic_bool;
    std::atomic<char> atomic_char;
    std::atomic<int> atomic_int;
    std::atomic<long> atomic_long;
    std::atomic<long long> atomic_ll;
    std::atomic<void*> atomic_void_ptr;
    std::atomic<int*> atomic_int_ptr;
    
    std::cout << "  bool:        " << (atomic_bool.is_lock_free() ? "YES" : "NO") << "\n";
    std::cout << "  char:        " << (atomic_char.is_lock_free() ? "YES" : "NO") << "\n";
    std::cout << "  int:         " << (atomic_int.is_lock_free() ? "YES" : "NO") << "\n";
    std::cout << "  long:        " << (atomic_long.is_lock_free() ? "YES" : "NO") << "\n";
    std::cout << "  long long:   " << (atomic_ll.is_lock_free() ? "YES" : "NO") << "\n";
    std::cout << "  void*:       " << (atomic_void_ptr.is_lock_free() ? "YES" : "NO") << "\n";
    std::cout << "  int*:        " << (atomic_int_ptr.is_lock_free() ? "YES" : "NO") << "\n";
    
    // Check using runtime function
    std::cout << "\nUsing std::atomic_is_lock_free():\n";
    std::cout << "  int:         " << (std::atomic_is_lock_free(&atomic_int) ? "YES" : "NO") << "\n";
    std::cout << "  long long:   " << (std::atomic_is_lock_free(&atomic_ll) ? "YES" : "NO") << "\n";
    std::cout << "  void*:       " << (std::atomic_is_lock_free(&atomic_void_ptr) ? "YES" : "NO") << "\n";
    
    // Check if always lock-free
    std::cout << "\nAdditional info:\n";
    std::cout << "  ATOMIC_BOOL_LOCK_FREE:     " << ATOMIC_BOOL_LOCK_FREE << "\n";
    std::cout << "  ATOMIC_CHAR_LOCK_FREE:     " << ATOMIC_CHAR_LOCK_FREE << "\n";
    std::cout << "  ATOMIC_SHORT_LOCK_FREE:    " << ATOMIC_SHORT_LOCK_FREE << "\n";
    std::cout << "  ATOMIC_INT_LOCK_FREE:      " << ATOMIC_INT_LOCK_FREE << "\n";
    std::cout << "  ATOMIC_LONG_LOCK_FREE:     " << ATOMIC_LONG_LOCK_FREE << "\n";
    std::cout << "  ATOMIC_LLONG_LOCK_FREE:    " << ATOMIC_LLONG_LOCK_FREE << "\n";
    std::cout << "  ATOMIC_POINTER_LOCK_FREE:  " << ATOMIC_POINTER_LOCK_FREE << "\n";
    std::cout << "\n(0 = never lock-free, 1 = sometimes lock-free, 2 = always lock-free)\n";
    
    return 0;
}

