def FlagsForFile( filename, **kwargs ):
      return {
          'flags': [ '-x', 'c++','-std=c++14','-Wall', '-Wextra', '-Werror','-isystem/usr/bin/../lib/gcc/x86_64-linux-gnu/5.4.0/../../../../include/c++/5.4.0','-isystem/usr/bin/../lib/gcc/x86_64-linux-gnu/5.4.0/../../../../include/x86_64-linux-gnu/c++/5.4.0','-isystem/usr/bin/../lib/gcc/x86_64-linux-gnu/5.4.0/../../../../include/c++/5.4.0/backward','-isystem/usr/local/include','-isystem/usr/lib/llvm-3.8/bin/../lib/clang/3.8.0/include','-isystem/usr/include/x86_64-linux-gnu','-isystem/usr/include' ],
      }
