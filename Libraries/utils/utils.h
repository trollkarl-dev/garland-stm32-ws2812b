#ifndef UTILS_H
#define UTILS_H

#define STATIC_ASSERT_MSG(COND,MSG) typedef char static_assertion_at_line_##MSG[(COND)?1:-1]
#define STATIC_ASSERT2(X,L) STATIC_ASSERT_MSG(X,L)
#define STATIC_ASSERT(COND) STATIC_ASSERT2(COND,__LINE__)

#endif /* UTILS_H */