#ifndef QI_CLIENT
#define QI_CLIENT

#include <string>

class QIClient
{
public:
    QIClient(const char* name);
    ~QIClient();

    void openSharedMemory();

    std::string getName() const { return m_name; }
    void* getBuffer() const { return m_shmem_ptr; }

protected:
    std::string m_name;

    size_t m_shmem_size;
    void* m_shmem_ptr;
};

#endif // QI_CLIENT
