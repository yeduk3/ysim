
template <typename T>
struct MemoryPool {
    std::vector<T> pool;
    size_t marker = 0;
    MemoryPool() {}
    MemoryPool(size_t N) : pool(N) {}
    //T& operator[](size_t i) { return pool[i]; }
    //T* data() { return pool.data(); }
    T* alloc(size_t count) {
        if(marker >= pool.size()) {
            std::cout << "[Pool] Memory overflow" << std::endl;
            exit(1);
        }
        T* ret = pool.data() + marker;
        marker += count;
        return ret;
    }
    T* zeros(size_t count) {
        T* ret = alloc(count);
        memset(ret, 0, count*sizeof(T));
        return ret;
    }
    T* allocFill(size_t count, T fill) {
        T* ret = alloc(count);
        std::fill(ret, ret+count, fill);
        return ret;
    }
};
void addTest_CPU() {
    std::cout << "----- Start: Addition Test -----" << std::endl;
    constexpr size_t num = 100000;
    std::cout << "----- Info: Vec size = (" << num << ") -----" << std::endl;
    MemoryPool<float> pool(num*10);

    auto start = std::chrono::steady_clock::now();
    VectorBase<CPU, float> x(pool.zeros(num), num);
    VectorBase<CPU, float> y(pool.zeros(num), num);
    VectorBase<CPU, float> c(pool.alloc(num), num);
    c.map() = x.map()+y.map();
    //std::cout << "Vector c: " << c.data << std::endl;
    auto end = std::chrono::steady_clock::now();
    auto diff = std::chrono::duration_cast<std::chrono::microseconds>(end-start);
    float dummy = c.map()[0];
    std::cout << "Ex1 on my Vec: " << diff.count() << " us" << std::endl;

    start = std::chrono::steady_clock::now();
    float* mem1 = pool.zeros(num);
    for(int i = 0; i < num; i++) *(mem1+i) = i;
    float* mem2 = pool.zeros(num);
    for(int i = 0; i < num; i++) *(mem2+i) = i+num;
    VectorBase<CPU, float> x1(mem1, num);
    VectorBase<CPU, float> x2(mem2, num);
    c.map() = x1.map()+x2.map();
    end = std::chrono::steady_clock::now();
    diff = std::chrono::duration_cast<std::chrono::microseconds>(end-start);
    dummy += c.map()[0];
    std::cout << "Ex2 on my Vec: " << diff.count() << " us" << std::endl;
    //std::cout << "Vector c1: " << c1.data << std::endl;

    start = std::chrono::steady_clock::now();
    Eigen::VectorXf xe1 = Eigen::VectorXf::Zero(num), xe2 = Eigen::VectorXf::Zero(num);
    Eigen::VectorXf ce1 = xe1+xe2;
    end = std::chrono::steady_clock::now();
    diff = std::chrono::duration_cast<std::chrono::microseconds>(end-start);
    std::cout << "Ex1 on Eigen: " << diff.count() << " us" << std::endl;
    dummy += ce1[0];

    start = std::chrono::steady_clock::now();
    Eigen::VectorXf xe3(num), xe4(num);
    for(int i = 0; i < num; i++) xe3.coeffRef(i) = i;
    for(int i = 0; i < num; i++) xe4.coeffRef(i) = i+num;
    Eigen::VectorXf ce2 = xe3+xe4;
    end = std::chrono::steady_clock::now();
    diff = std::chrono::duration_cast<std::chrono::microseconds>(end-start);
    dummy += ce2[0];
    std::cout << "Ex2 on Eigen: " << diff.count() << " us" << std::endl;

    std::cout<< "dummy: " << dummy << std::endl;
    std::cout << "----- End: Addition Test -----" << std::endl << std::endl;
}
void mulTest_CPU() {
    std::cout << "----- Start: MV Multiplication Test -----" << std::endl;
    constexpr size_t num = 300;
    std::cout << "----- Info: Mat size = (" << num << "x" << num << ") * Vec size = (" << num << ") -----" << std::endl;
    constexpr size_t num2 = num*num;
    MemoryPool<float> pool(num2*10);

    auto start = std::chrono::steady_clock::now();
    Matrix<CPU, float> m(pool.zeros(num2), num, num);
    VectorBase<CPU, float> a(pool.zeros(num), num);
    VectorBase<CPU, float> b(pool.alloc(num), num);
    b.map() = m.map()*a.map();
    auto end = std::chrono::steady_clock::now();
    auto diff = std::chrono::duration_cast<std::chrono::microseconds>(end-start);
    float dummy = b.map()[0];
    std::cout << "Ex1 on my Mat*Vec: " << diff.count() << " us" << std::endl;

    start = std::chrono::steady_clock::now();
    float* mem1 = pool.zeros(num2);
    for(size_t i = 0; i < num2; i++) *(mem1+i) = i;
    Matrix<CPU, float> m2(mem1, num, num);
    float* mem2 = pool.zeros(num);
    for(size_t i = 0; i < num; i++) *(mem2+i) = i;
    VectorBase<CPU, float> a2(mem2, num);
    b.map() = m2.map()*a2.map();
    end = std::chrono::steady_clock::now();
    diff = std::chrono::duration_cast<std::chrono::microseconds>(end-start);
    dummy = b.map()[0];
    std::cout << "Ex2 on my Mat*Vec: " << diff.count() << " us" << std::endl;


    // --- 여기서부터 채워진 Eigen 테스트 부분입니다 ---
    start = std::chrono::steady_clock::now();
    Eigen::MatrixXf me1 = Eigen::MatrixXf::Zero(num, num);
    Eigen::VectorXf ae1 = Eigen::VectorXf::Zero(num);
    Eigen::VectorXf be1 = me1 * ae1;
    end = std::chrono::steady_clock::now();
    diff = std::chrono::duration_cast<std::chrono::microseconds>(end-start);
    dummy += be1[0];
    std::cout << "Ex1 on Eigen: " << diff.count() << " us" << std::endl;

    start = std::chrono::steady_clock::now();
    Eigen::MatrixXf me2(num, num);
    // Eigen의 내부 1차원 배열 포인터(data())를 이용해 mem1, mem2와 완벽히 동일한 방식으로 초기화합니다.
    for(size_t i = 0; i < num2; i++) *(me2.data() + i) = i;
    Eigen::VectorXf ae2(num);
    for(size_t i = 0; i < num; i++) *(ae2.data() + i) = i;
    Eigen::VectorXf be2 = me2 * ae2;
    end = std::chrono::steady_clock::now();
    diff = std::chrono::duration_cast<std::chrono::microseconds>(end-start);
    dummy += be2[0];
    std::cout << "Ex2 on Eigen: " << diff.count() << " us" << std::endl;

    // 컴파일러의 Dead Code Elimination 최적화를 막기 위한 더미 출력
    std::cout << "dummy: " << dummy << std::endl;
    std::cout << "----- End: MV Multiplication Test -----" << std::endl << std::endl;
}
#include <random>
void sparseMatrixMulTest_CPU() {
    std::cout << "----- Start: SMV Multiplication Test -----" << std::endl;
    constexpr size_t size = 10000;
    constexpr float nnzratio = 0.1;
    constexpr size_t nnz = size * size * nnzratio; 
    std::cout << "----- Info: Sparse Mat size = (" << size << "x" << size << ") * Vec size = (" << size << "), non-zeros ratio = " << nnzratio << " -----" << std::endl;
    // 1000 * 1000 의 10% = 100,000 개의 0이 아닌 요소(Non-zeros)

    // 1. 임의의 Triplet 데이터 생성 (타이머 바깥에서 준비)
    std::vector<Eigen::Triplet<float>> triplets;
    triplets.reserve(nnz);
    
    // 고정 시드를 사용하여 매번 동일한 패턴의 난수 생성
    std::mt19937 gen(42); 
    std::uniform_int_distribution<int> dist(0, size - 1);
    std::uniform_real_distribution<float> val_dist(-1.0f, 1.0f);

    for (size_t i = 0; i < nnz; ++i) {
        triplets.emplace_back(dist(gen), dist(gen), val_dist(gen));
    }

    MemoryPool<float> pool(size * 10);
    float dummy = 0.0f;

    // ==========================================
    // Ex1: 내 커스텀 SparseMatrix & Vector 연산
    // ==========================================
    // 데이터 세팅
    SparseMatrix<CPU, float> mySm(triplets, size, size);
    float* memX = pool.zeros(size);
    for(int i = 0; i < size; i++) memX[i] = 1.0f; // 벡터를 1.0으로 초기화
    VectorBase<CPU, float> myX(memX, size);
    VectorBase<CPU, float> myRes(pool.alloc(size), size);

    auto start = std::chrono::steady_clock::now();
    
    // 행렬-벡터 곱셈 실행
    myRes.map() = mySm.map() * myX.map();
    
    auto end = std::chrono::steady_clock::now();
    auto diff = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Sparse * Vec on my Struct: " << diff.count() << " us" << std::endl;
    dummy += myRes.map()[0];

    // ==========================================
    // Ex2: 순수 Eigen SparseMatrix & Vector 연산
    // ==========================================
    // 데이터 세팅
    Eigen::SparseMatrix<float> eigenSm(size, size);
    eigenSm.setFromTriplets(triplets.begin(), triplets.end());
    Eigen::VectorXf eigenX = Eigen::VectorXf::Ones(size);

    start = std::chrono::steady_clock::now();
    
    // 행렬-벡터 곱셈 실행
    Eigen::VectorXf eigenRes = eigenSm * eigenX;
    
    end = std::chrono::steady_clock::now();
    diff = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Sparse * Vec on pure Eigen: " << diff.count() << " us" << std::endl;
    dummy += eigenRes[0];

    std::cout << "dummy: " << dummy << std::endl;
    std::cout << "----- End: SMV Multiplication Test -----" << std::endl << std::endl;
}

void test() {
    addTest_CPU();
    mulTest_CPU();
    sparseMatrixMulTest_CPU();
}

