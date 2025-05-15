#ifndef CIRCULARBUF_H
#define CIRCULARBUF_H



template<typename TYPE, size_t SIZE>
class circular_buf 
{
    public:
        circular_buf() : buffer(new TYPE[SIZE]), head(0), tail(0) {}

        ~circular_buf() 
        {
            delete[] buffer;
        }

        void push(const TYPE& item) 
        {
            if (is_full()) {
                return;     // Don't store anything if the buffer is full
            }
            buffer[head] = item;
            head = (head + 1) % SIZE;
        }

        TYPE pop() 
        {
            auto item = buffer[tail];
            if (!is_empty())
                tail = (tail + 1) % SIZE;   // Only move the tail if there was actually data
            return item;
        }

        bool is_empty() const 
        {
            return head == tail;
        }

        bool is_full() const 
        {
            return (head + 1) % SIZE == tail;
        }

        uint item_count() 
        {
            return head - tail;
        }

    private:
        TYPE* buffer;
        size_t head, tail;
};

#endif // circular_buf_H