#pragma once
#include <vector>
#include <memory>
#include <string>
#include <functional>
#include <unordered_map>
#include <stdexcept>
#include <type_traits>
#include <optional>

// ============================================================
//  Behavior Tree v2 — Single-header C++ (C++17)
//
//  개선 사항:
//    1. Blackboard: get() → std::optional 반환으로 예외 없는 API 추가
//    2. Parallel: 매직 넘버 0 제거 → 생성자에서 즉시 보정
//    3. Repeat/Retry: 단일 tick 내 다중 실행 방지 (per-tick 1회 진행)
//    4. Builder: end() 누락 감지 (소멸자 assert + buildChecked())
//    5. ControlNode::addChild: blackboard_ nullptr 전파 안전 처리
//    6. setStatus() 반환값 활용 유지 (강점 보존)
//    7. Decorator 자동 닫기 재귀 유지 (강점 보존)
// ============================================================

namespace BT {

// ----------------------------------------------------------------
//  Status
// ----------------------------------------------------------------
enum class Status { Success, Failure, Running };

inline const char* toString(Status s) noexcept {
    switch (s) {
        case Status::Success: return "SUCCESS";
        case Status::Failure: return "FAILURE";
        case Status::Running: return "RUNNING";
    }
    return "UNKNOWN";
}

// ----------------------------------------------------------------
//  Blackboard
//  개선: tryGet → std::optional<T> get() 추가
//       기존 throw 버전은 getOrThrow()로 명칭 명확화
// ----------------------------------------------------------------
class Blackboard {
public:
    using Ptr = std::shared_ptr<Blackboard>;
    static Ptr create() { return std::make_shared<Blackboard>(); }

    template<typename T>
    void set(const std::string& key, T value) {
        data_[key] = std::make_shared<Entry<T>>(std::move(value));
    }

    // [개선 1] 예외 없는 API — std::optional 반환
    // 키 없음 / 타입 불일치 → std::nullopt
    template<typename T>
    std::optional<T> get(const std::string& key) const {
        auto it = data_.find(key);
        if (it == data_.end()) return std::nullopt;
        auto* e = dynamic_cast<Entry<T>*>(it->second.get());
        if (!e) return std::nullopt;
        return e->value;
    }

    // 반드시 값이 있어야 하는 경우 — 명시적으로 예외 가능성 표현
    template<typename T>
    T getOrThrow(const std::string& key) const {
        auto result = get<T>(key);
        if (!result)
            throw std::runtime_error("Blackboard: key not found or type mismatch: " + key);
        return *result;
    }

    bool has(const std::string& key) const { return data_.count(key) > 0; }

    void erase(const std::string& key) { data_.erase(key); }

private:
    struct EntryBase { virtual ~EntryBase() = default; };
    template<typename T>
    struct Entry : EntryBase {
        T value;
        explicit Entry(T v) : value(std::move(v)) {}
    };
    std::unordered_map<std::string, std::shared_ptr<EntryBase>> data_;
};

// ----------------------------------------------------------------
//  TreeNode
// ----------------------------------------------------------------
class TreeNode {
public:
    explicit TreeNode(std::string name) : name_(std::move(name)) {}
    virtual ~TreeNode() = default;

    virtual Status tick() = 0;
    virtual void halt() {}
    virtual void initialize() {}

    const std::string& name()   const noexcept { return name_; }
    Status             status() const noexcept { return status_; }

    virtual void setBlackboard(Blackboard::Ptr bb) { blackboard_ = std::move(bb); }

    // [강점 보존] nullptr 역참조 전에 검사
    Blackboard& blackboard() {
        if (!blackboard_)
            throw std::runtime_error("TreeNode '" + name_ + "': blackboard not set");
        return *blackboard_;
    }

protected:
    // [강점 보존] 반환값 활용 패턴: return setStatus(...)
    Status setStatus(Status s) noexcept { return status_ = s; }

    std::string     name_;
    Status          status_ = Status::Failure;
    Blackboard::Ptr blackboard_;
};

using NodePtr = std::shared_ptr<TreeNode>;

// ----------------------------------------------------------------
//  LeafNode
// ----------------------------------------------------------------
class LeafNode : public TreeNode {
public:
    using TreeNode::TreeNode;
};

// ----------------------------------------------------------------
//  ControlNode
//  [개선 5] addChild: blackboard_가 nullptr여도 나중에 setBlackboard가
//           전파하므로 안전하지만, 혼용 시 위험을 막기 위해
//           setBlackboard를 override해서 후속 addChild도 항상 전파
// ----------------------------------------------------------------
class ControlNode : public TreeNode {
public:
    using TreeNode::TreeNode;

    void addChild(NodePtr child) {
        if (!child) throw std::invalid_argument("addChild: null child");
        // blackboard_가 이미 설정돼 있으면 즉시 전파; 없으면 나중에 setBlackboard가 전파
        if (blackboard_) child->setBlackboard(blackboard_);
        children_.push_back(std::move(child));
    }

    void setBlackboard(Blackboard::Ptr bb) {
        blackboard_ = bb;
        for (auto& c : children_) c->setBlackboard(bb);
    }

    void halt() override {
        for (auto& c : children_) c->halt();
    }

    const std::vector<NodePtr>& children() const noexcept { return children_; }

protected:
    std::vector<NodePtr> children_;
};

// ----------------------------------------------------------------
//  DecoratorNode
// ----------------------------------------------------------------
class DecoratorNode : public TreeNode {
public:
    explicit DecoratorNode(std::string name, NodePtr child = nullptr)
        : TreeNode(std::move(name)), child_(std::move(child)) {}

    void setChild(NodePtr child) {
        if (!child) throw std::invalid_argument("setChild: null child");
        child_ = std::move(child);
    }

    void setBlackboard(Blackboard::Ptr bb) {
        blackboard_ = bb;
        if (child_) child_->setBlackboard(bb);
    }

    void halt() override { if (child_) child_->halt(); }

protected:
    NodePtr child_;
};

// ================================================================
//  Composite Nodes
// ================================================================

class Sequence : public ControlNode {
public:
    explicit Sequence(std::string name = "Sequence") : ControlNode(std::move(name)) {}

    Status tick() override {
        for (auto& child : children_) {
            Status s = child->tick();
            if (s != Status::Success) return setStatus(s);
        }
        return setStatus(Status::Success);
    }
};

// ReactiveSequence는 Sequence와 동일 (매 tick 처음부터) — 명시적 별칭
class ReactiveSequence : public Sequence {
public:
    explicit ReactiveSequence(std::string name = "ReactiveSequence")
        : Sequence(std::move(name)) {}
};

class MemorySequence : public ControlNode {
public:
    explicit MemorySequence(std::string name = "MemorySequence")
        : ControlNode(std::move(name)) {}

    Status tick() override {
        for (size_t i = cursor_; i < children_.size(); ++i) {
            Status s = children_[i]->tick();
            if (s == Status::Running) { cursor_ = i; return setStatus(s); }
            if (s == Status::Failure) { cursor_ = 0; return setStatus(s); }
        }
        cursor_ = 0;
        return setStatus(Status::Success);
    }

    void halt() override { cursor_ = 0; ControlNode::halt(); }

private:
    size_t cursor_ = 0;
};

class Selector : public ControlNode {
public:
    explicit Selector(std::string name = "Selector") : ControlNode(std::move(name)) {}

    Status tick() override {
        for (auto& child : children_) {
            Status s = child->tick();
            if (s != Status::Failure) return setStatus(s);
        }
        return setStatus(Status::Failure);
    }
};

class MemorySelector : public ControlNode {
public:
    explicit MemorySelector(std::string name = "MemorySelector")
        : ControlNode(std::move(name)) {}

    Status tick() override {
        for (size_t i = cursor_; i < children_.size(); ++i) {
            Status s = children_[i]->tick();
            if (s == Status::Running) { cursor_ = i; return setStatus(s); }
            if (s == Status::Success) { cursor_ = 0; return setStatus(s); }
        }
        cursor_ = 0;
        return setStatus(Status::Failure);
    }

    void halt() override { cursor_ = 0; ControlNode::halt(); }

private:
    size_t cursor_ = 0;
};

// ----------------------------------------------------------------
//  Parallel
//  [개선 2] 매직 넘버 0 제거
//  생성자에서 successThreshold가 kAll이면 addChild 이후에도 반영되어야
//  하므로 "kAll 플래그"를 별도로 보존하고 tick() 진입 시 children_.size()로 계산
// ----------------------------------------------------------------
class Parallel : public ControlNode {
public:
    static constexpr size_t kAll = SIZE_MAX;  // "모든 자식이 성공해야 함"을 명시적으로 표현

    // successThreshold: kAll = 전체, N = N개 이상
    // failureThreshold: 기본 1 = 하나라도 실패하면 즉시 Failure
    explicit Parallel(std::string name    = "Parallel",
                      size_t successThreshold = kAll,
                      size_t failureThreshold = 1)
        : ControlNode(std::move(name))
        , successThreshold_(successThreshold)
        , failureThreshold_(failureThreshold) {}

    Status tick() override {
        const size_t N  = children_.size();
        // tick() 진입 시점에 kAll → 실제 크기로 결정 (자식 개수가 변해도 안전)
        const size_t st = (successThreshold_ == kAll) ? N : successThreshold_;

        size_t successes = 0, failures = 0;
        for (auto& child : children_) {
            Status s = child->tick();
            if (s == Status::Success) ++successes;
            else if (s == Status::Failure) ++failures;
            if (successes >= st)               return setStatus(Status::Success);
            if (failures  >= failureThreshold_) return setStatus(Status::Failure);
        }
        return setStatus(Status::Running);
    }

private:
    size_t successThreshold_;
    size_t failureThreshold_;
};

// ================================================================
//  Decorator Nodes
// ================================================================

class Inverter : public DecoratorNode {
public:
    explicit Inverter(NodePtr child = nullptr, std::string name = "Inverter")
        : DecoratorNode(std::move(name), std::move(child)) {}

    Status tick() override {
        Status s = child_->tick();
        if (s == Status::Success) return setStatus(Status::Failure);
        if (s == Status::Failure) return setStatus(Status::Success);
        return setStatus(s);
    }
};

class ForceSuccess : public DecoratorNode {
public:
    explicit ForceSuccess(NodePtr child = nullptr, std::string name = "ForceSuccess")
        : DecoratorNode(std::move(name), std::move(child)) {}

    Status tick() override {
        Status s = child_->tick();
        if (s == Status::Running) return setStatus(s);
        return setStatus(Status::Success);
    }
};

class ForceFailure : public DecoratorNode {
public:
    explicit ForceFailure(NodePtr child = nullptr, std::string name = "ForceFailure")
        : DecoratorNode(std::move(name), std::move(child)) {}

    Status tick() override {
        Status s = child_->tick();
        if (s == Status::Running) return setStatus(s);
        return setStatus(Status::Failure);
    }
};

// ----------------------------------------------------------------
//  Repeat
//  [개선 3] 단일 tick 내 다중 실행 방지
//  핵심 변경: while → if
//  자식이 즉시 Success를 반환해도 한 tick에 1회만 카운트를 올리고
//  다음 tick을 기다립니다. 자식이 Running을 반환하면 그대로 전달합니다.
// ----------------------------------------------------------------
class Repeat : public DecoratorNode {
public:
    static constexpr int kForever = -1;

    explicit Repeat(NodePtr child = nullptr,
                    int count     = kForever,
                    std::string name = "Repeat")
        : DecoratorNode(std::move(name), std::move(child))
        , maxCount_(count) {}

    Status tick() override {
        if (maxCount_ != kForever && counter_ >= maxCount_) {
            counter_ = 0;
            return setStatus(Status::Success);
        }

        Status s = child_->tick();  // 한 tick에 1회만 실행
        if (s == Status::Failure) {
            counter_ = 0;
            return setStatus(Status::Failure);
        }
        if (s == Status::Running) {
            return setStatus(Status::Running);
        }
        // Success: 카운트 증가 후 완료 여부 판단
        ++counter_;
        if (maxCount_ != kForever && counter_ >= maxCount_) {
            counter_ = 0;
            return setStatus(Status::Success);
        }
        return setStatus(Status::Running);  // 아직 반복 횟수 남음 → 다음 tick 대기
    }

    void halt() override { counter_ = 0; DecoratorNode::halt(); }

private:
    int maxCount_;
    int counter_ = 0;
};

// ----------------------------------------------------------------
//  Retry
//  [개선 3] 단일 tick 내 다중 시도 방지
//  while → if: 실패해도 한 tick에 1회만 시도하고 다음 tick을 기다립니다.
// ----------------------------------------------------------------
class Retry : public DecoratorNode {
public:
    explicit Retry(NodePtr child     = nullptr,
                   int maxAttempts   = 3,
                   std::string name  = "Retry")
        : DecoratorNode(std::move(name), std::move(child))
        , maxAttempts_(maxAttempts) {}

    Status tick() override {
        if (attempts_ >= maxAttempts_) {
            attempts_ = 0;
            return setStatus(Status::Failure);
        }

        Status s = child_->tick();  // 한 tick에 1회만 시도
        if (s == Status::Success) {
            attempts_ = 0;
            return setStatus(Status::Success);
        }
        if (s == Status::Running) {
            return setStatus(Status::Running);
        }
        // Failure: 시도 횟수 증가
        ++attempts_;
        if (attempts_ >= maxAttempts_) {
            attempts_ = 0;
            return setStatus(Status::Failure);
        }
        return setStatus(Status::Running);  // 재시도 가능 → 다음 tick 대기
    }

    void halt() override { attempts_ = 0; DecoratorNode::halt(); }

private:
    int maxAttempts_;
    int attempts_ = 0;
};

// ================================================================
//  Leaf Nodes
// ================================================================

class ActionNode : public LeafNode {
public:
    using Func = std::function<Status()>;

    ActionNode(std::string name, Func fn)
        : LeafNode(std::move(name)), fn_(std::move(fn)) {
        if (!fn_) throw std::invalid_argument("ActionNode: null function");
    }

    Status tick() override { return setStatus(fn_()); }

private:
    Func fn_;
};

class ConditionNode : public LeafNode {
public:
    using Pred = std::function<bool()>;

    ConditionNode(std::string name, Pred pred)
        : LeafNode(std::move(name)), pred_(std::move(pred)) {
        if (!pred_) throw std::invalid_argument("ConditionNode: null predicate");
    }

    Status tick() override {
        return setStatus(pred_() ? Status::Success : Status::Failure);
    }

private:
    Pred pred_;
};

// ================================================================
//  BehaviorTree
// ================================================================
class BehaviorTree {
public:
    explicit BehaviorTree(NodePtr root, Blackboard::Ptr bb = nullptr)
        : root_(std::move(root))
        , blackboard_(bb ? std::move(bb) : Blackboard::create())
    {
        if (!root_) throw std::invalid_argument("BehaviorTree: null root");
        root_->setBlackboard(blackboard_);
        root_->initialize();
    }

    Status tick()  { return root_->tick(); }
    void   halt()  { root_->halt(); }

    Blackboard& blackboard() noexcept { return *blackboard_; }
    NodePtr     root()       noexcept { return root_; }

private:
    NodePtr         root_;
    Blackboard::Ptr blackboard_;
};

// ================================================================
//  Builder — 스택 기반 Fluent DSL
//  [개선 4] end() 누락 감지:
//    - 소멸자에서 스택에 미완성 노드가 남아있으면 stderr 경고
//    - buildChecked()는 스택이 깔끔하지 않으면 예외 발생
// ================================================================
class Builder {
public:
    Builder() = default;

    // [개선 4] 소멸자에서 미완성 노드 경고
    ~Builder() {
        if (stack_.size() > 1) {
            // 예외를 던질 수 없으므로 stderr 경고
            // build() 없이 Builder가 소멸되는 경우도 커버
            fprintf(stderr,
                "[BT::Builder] WARNING: %zu node(s) were never closed with end().\n"
                "  Unclosed nodes (bottom→top):\n",
                stack_.size() - 1);
            for (size_t i = 1; i < stack_.size(); ++i)
                fprintf(stderr, "    - %s\n", stack_[i]->name().c_str());
        }
    }

    // 복사/이동 금지 (스택 상태를 공유하면 안 됨)
    Builder(const Builder&)            = delete;
    Builder& operator=(const Builder&) = delete;
    Builder(Builder&&)                 = default;
    Builder& operator=(Builder&&)      = default;

    Builder& sequence(const std::string& name = "Sequence") {
        push(std::make_shared<Sequence>(name)); return *this;
    }
    Builder& memSequence(const std::string& name = "MemorySequence") {
        push(std::make_shared<MemorySequence>(name)); return *this;
    }
    Builder& selector(const std::string& name = "Selector") {
        push(std::make_shared<Selector>(name)); return *this;
    }
    Builder& memSelector(const std::string& name = "MemorySelector") {
        push(std::make_shared<MemorySelector>(name)); return *this;
    }
    Builder& parallel(size_t sucThresh = Parallel::kAll,
                      size_t failThresh = 1,
                      const std::string& name = "Parallel") {
        push(std::make_shared<Parallel>(name, sucThresh, failThresh)); return *this;
    }
    Builder& inverter(const std::string& name = "Inverter") {
        push(std::make_shared<Inverter>(nullptr, name)); return *this;
    }
    Builder& forceSuccess(const std::string& name = "ForceSuccess") {
        push(std::make_shared<ForceSuccess>(nullptr, name)); return *this;
    }
    Builder& repeat(int count = Repeat::kForever, const std::string& name = "Repeat") {
        push(std::make_shared<Repeat>(nullptr, count, name)); return *this;
    }
    Builder& retry(int attempts = 3, const std::string& name = "Retry") {
        push(std::make_shared<Retry>(nullptr, attempts, name)); return *this;
    }
    Builder& action(const std::string& name, ActionNode::Func fn) {
        addLeaf(std::make_shared<ActionNode>(name, std::move(fn))); return *this;
    }
    Builder& condition(const std::string& name, ConditionNode::Pred pred) {
        addLeaf(std::make_shared<ConditionNode>(name, std::move(pred))); return *this;
    }

    Builder& end() {
        if (stack_.size() <= 1) return *this;
        auto finished = stack_.back(); stack_.pop_back();
        attachToParent(finished);
        return *this;
    }

    // 관대한 build(): end() 누락을 자동으로 닫아줌 (v1 동작 유지)
    std::shared_ptr<BehaviorTree> build(Blackboard::Ptr bb = nullptr) {
        if (stack_.empty()) throw std::runtime_error("Builder: empty tree");
        while (stack_.size() > 1) end();
        auto tree = std::make_shared<BehaviorTree>(stack_.front(), std::move(bb));
        stack_.clear();
        return tree;
    }

    // [개선 4] 엄격한 build(): end() 누락 시 예외 발생
    std::shared_ptr<BehaviorTree> buildChecked(Blackboard::Ptr bb = nullptr) {
        if (stack_.empty()) throw std::runtime_error("Builder: empty tree");
        if (stack_.size() > 1) {
            std::string msg = "Builder::buildChecked: " +
                std::to_string(stack_.size() - 1) +
                " node(s) not closed with end():";
            for (size_t i = 1; i < stack_.size(); ++i)
                msg += "\n  - " + stack_[i]->name();
            throw std::runtime_error(msg);
        }
        auto tree = std::make_shared<BehaviorTree>(stack_.front(), std::move(bb));
        stack_.clear();
        return tree;
    }

private:
    std::vector<NodePtr> stack_;

    void push(NodePtr n) {
        stack_.push_back(std::move(n));
    }

    void addLeaf(NodePtr leaf) {
        if (stack_.empty()) { stack_.push_back(std::move(leaf)); return; }
        attachToParent(std::move(leaf));
    }

    // [강점 보존] Decorator 자동 닫기 재귀
    void attachToParent(NodePtr child) {
        if (stack_.empty()) {
            // 붙일 부모가 없다 = child가 루트가 됨
            stack_.push_back(std::move(child));
            return;
        }
        NodePtr parent = stack_.back();
        if (auto* c = dynamic_cast<ControlNode*>(parent.get())) {
            c->addChild(std::move(child));
        } else if (auto* d = dynamic_cast<DecoratorNode*>(parent.get())) {
            d->setChild(std::move(child));
            // Decorator는 자식이 하나 → 채워지는 즉시 자동으로 닫기
            stack_.pop_back();
            attachToParent(std::move(parent));  // 재귀: Decorator 자체를 상위에 붙이기
        }
    }
};

} // namespace BT
