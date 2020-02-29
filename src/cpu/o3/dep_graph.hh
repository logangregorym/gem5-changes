/*
 * Copyright (c) 2012 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2006 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Kevin Lim
 */

#ifndef __CPU_O3_DEP_GRAPH_HH__
#define __CPU_O3_DEP_GRAPH_HH__

#include <utility>

#include "cpu/o3/comm.hh"

using namespace std;

/** Node in a linked list. */
template <class DynInstPtr>
class DependencyEntry
{
  public:
    DependencyEntry()
        : inst(NULL), next(NULL)
    { }

    DynInstPtr inst;
    //Might want to include data about what arch. register the
    //dependence is waiting on.
    DependencyEntry<DynInstPtr> *next;
};

/** Array of linked list that maintains the dependencies between
 * producing instructions and consuming instructions.  Each linked
 * list represents a single physical register, having the future
 * producer of the register's value, and all consumers waiting on that
 * value on the list.  The head node of each linked list represents
 * the producing instruction of that register.  Instructions are put
 * on the list upon reaching the IQ, and are removed from the list
 * either when the producer completes, or the instruction is squashed.
*/
template <class DynInstPtr>
class DependencyGraph
{
  public:
    typedef DependencyEntry<DynInstPtr> DepEntry;

    /** Default construction.  Must call resize() prior to use. */
    DependencyGraph()
        : numEntries(0), memAllocCounter(0), nodesTraversed(0), nodesRemoved(0)
    { }

    ~DependencyGraph();

    /** Resize the dependency graph to have num_entries registers. */
    void resize(int num_entries);

    /** Clears all of the linked lists. */
    void reset();

    /** Inserts an instruction to be dependent on the given index. */
    void insert(PhysRegIndex idx, DynInstPtr &new_inst);

    /** Sets the producing instruction of a given register. */
    void setInst(PhysRegIndex idx, DynInstPtr &new_inst)
    { dependGraph[idx].inst = new_inst; }

    /** Clears the producing instruction. */
    void clearInst(PhysRegIndex idx)
    { dependGraph[idx].inst = NULL; }

    /** Removes an instruction from a single linked list. */
    void remove(PhysRegIndex idx, DynInstPtr &inst_to_remove);

    /** Removes and returns the newest dependent of a specific register. */
    DynInstPtr pop(PhysRegIndex idx);

    /** Checks if the entire dependency graph is empty. */
    bool empty() const;

    /** Checks if there are any dependents on a specific register. */
    bool empty(PhysRegIndex idx) const { return !dependGraph[idx].next; }

    /** Debugging function to dump out the dependency graph.
     */
    void dump();

    unsigned countDependentsOf(DynInstPtr &inst);

    unsigned countDependentsOf(PhysRegIndex idx, uint8_t recursiveDepth, vector<PhysRegIndex> &checked);

    vector<pair<DynInstPtr,unsigned>> getDependentsOf(DynInstPtr &inst);

    vector<pair<DynInstPtr,unsigned>> getDependentsOf(PhysRegIndex idx, uint8_t recursiveDepth, vector<PhysRegIndex> &checked);

  private:
    /** Array of linked lists.  Each linked list is a list of all the
     *  instructions that depend upon a given register.  The actual
     *  register's index is used to index into the graph; ie all
     *  instructions in flight that are dependent upon r34 will be
     *  in the linked list of dependGraph[34].
     */
    DepEntry *dependGraph;

    /** Number of linked lists; identical to the number of registers. */
    int numEntries;

    // Debug variable, remove when done testing.
    unsigned memAllocCounter;

  public:
    // Debug variable, remove when done testing.
    uint64_t nodesTraversed;

    // Debug variable, remove when done testing.
    uint64_t nodesRemoved;

    // When counting dependencies, how deep to go?
    unsigned maxDependencyRecursion = 50;
};

template <class DynInstPtr>
DependencyGraph<DynInstPtr>::~DependencyGraph()
{
    delete [] dependGraph;
}

template <class DynInstPtr>
void
DependencyGraph<DynInstPtr>::resize(int num_entries)
{
    numEntries = num_entries;
    dependGraph = new DepEntry[numEntries];
}

template <class DynInstPtr>
void
DependencyGraph<DynInstPtr>::reset()
{
    // Clear the dependency graph
    DepEntry *curr;
    DepEntry *prev;

    for (int i = 0; i < numEntries; ++i) {
        curr = dependGraph[i].next;

        while (curr) {
            memAllocCounter--;

            prev = curr;
            curr = prev->next;
            prev->inst = NULL;

            delete prev;
        }

        if (dependGraph[i].inst) {
            dependGraph[i].inst = NULL;
        }

        dependGraph[i].next = NULL;
    }
}

template <class DynInstPtr>
void
DependencyGraph<DynInstPtr>::insert(PhysRegIndex idx, DynInstPtr &new_inst)
{
    //Add this new, dependent instruction at the head of the dependency
    //chain.

    // First create the entry that will be added to the head of the
    // dependency chain.
    DepEntry *new_entry = new DepEntry;
    new_entry->next = dependGraph[idx].next;
    new_entry->inst = new_inst;

    // Then actually add it to the chain.
    dependGraph[idx].next = new_entry;

    ++memAllocCounter;
}


template <class DynInstPtr>
void
DependencyGraph<DynInstPtr>::remove(PhysRegIndex idx,
                                    DynInstPtr &inst_to_remove)
{
    DepEntry *prev = &dependGraph[idx];
    DepEntry *curr = dependGraph[idx].next;

    // Make sure curr isn't NULL.  Because this instruction is being
    // removed from a dependency list, it must have been placed there at
    // an earlier time.  The dependency chain should not be empty,
    // unless the instruction dependent upon it is already ready.
    if (curr == NULL) {
        return;
    }

    nodesRemoved++;

    // Find the instruction to remove within the dependency linked list.
    while (curr->inst != inst_to_remove) {
        prev = curr;
        curr = curr->next;
        nodesTraversed++;

        assert(curr != NULL);
    }

    // Now remove this instruction from the list.
    prev->next = curr->next;

    --memAllocCounter;

    // Could push this off to the destructor of DependencyEntry
    curr->inst = NULL;

    delete curr;
}

template <class DynInstPtr>
DynInstPtr
DependencyGraph<DynInstPtr>::pop(PhysRegIndex idx)
{
    DepEntry *node;
    node = dependGraph[idx].next;
    DynInstPtr inst = NULL;
    if (node) {
        inst = node->inst;
        dependGraph[idx].next = node->next;
        node->inst = NULL;
        memAllocCounter--;
        delete node;
    }
    return inst;
}

template <class DynInstPtr>
bool
DependencyGraph<DynInstPtr>::empty() const
{
    for (int i = 0; i < numEntries; ++i) {
        if (!empty(i))
            return false;
    }
    return true;
}

template <class DynInstPtr>
void
DependencyGraph<DynInstPtr>::dump()
{
    DepEntry *curr;

    for (int i = 0; i < numEntries; ++i)
    {
        curr = &dependGraph[i];

        if (curr->inst) {
            cprintf("dependGraph[%i]: producer: %s [sn:%lli] consumer: ",
                    i, curr->inst->pcState(), curr->inst->seqNum);
        } else {
            cprintf("dependGraph[%i]: No producer. consumer: ", i);
        }

        while (curr->next != NULL) {
            curr = curr->next;

            cprintf("%s [sn:%lli] ",
                    curr->inst->pcState(), curr->inst->seqNum);
        }

        cprintf("\n");
    }
    cprintf("memAllocCounter: %i\n", memAllocCounter);
}

template <class DynInstPtr>
unsigned
DependencyGraph<DynInstPtr>::countDependentsOf(DynInstPtr &inst) {
    unsigned total = 0;
    vector<PhysRegIndex> checked = vector<PhysRegIndex>();
    for (int i = 0; i < inst->numDestRegs(); i++) {
        PhysRegIndex idx = inst->renamedDestRegIdx(i)->flatIndex();
        checked.push_back(idx);
        total += countDependentsOf(idx, 0, checked);
    }
    return total;
}

template <class DynInstPtr>
vector<pair<DynInstPtr,unsigned>>
DependencyGraph<DynInstPtr>::getDependentsOf(DynInstPtr &inst) {
    vector<pair<DynInstPtr,unsigned>> dependentList = vector<pair<DynInstPtr,unsigned>>();
    vector<PhysRegIndex> checked = vector<PhysRegIndex>();
    vector<DynInstPtr> seen = vector<DynInstPtr>();
    seen.push_back(inst);
    dependentList.push_back(make_pair(inst, 0));
    for (int i = 0; i < inst->numDestRegs(); i++) {
        PhysRegIndex idx = inst->renamedDestRegIdx(i)->flatIndex();
        checked.push_back(idx);
        if (inst->renamedDestRegIdx(i)->isIntPhysReg() || inst->renamedDestRegIdx(i)->isFloatPhysReg()) {
            vector<pair<DynInstPtr,unsigned>> ret = getDependentsOf(idx, 0, checked);
            for (int i = 0; i < ret.size(); i++) {
                if (!count(seen.begin(), seen.end(), ret[i].first)) {
                    dependentList.push_back(ret[i]);
                    seen.push_back(ret[i].first);
                }
            }
        }
    }
    return dependentList;
}

template <class DynInstPtr>
unsigned
DependencyGraph<DynInstPtr>::countDependentsOf(PhysRegIndex idx, uint8_t recursiveDepth, vector<PhysRegIndex> &checked) {
    if (recursiveDepth >= maxDependencyRecursion) {
        return 0;
    }
    unsigned total = 0;
    DepEntry *curr = &dependGraph[idx];
    while (curr != NULL) {
        if (curr->inst) {
            total++;
            DynInstPtr inst = curr->inst;
            for (int i = 0; i < inst->numDestRegs(); i++) {
                PhysRegIndex idx = inst->renamedDestRegIdx(i)->flatIndex();
                if (!count(checked.begin(), checked.end(), idx)) {
                    checked.push_back(idx);
                    total += countDependentsOf(idx, recursiveDepth + 1, checked);
                }
            }
        }
        curr = curr->next;
    }
    return total;
}

template <class DynInstPtr>
vector<pair<DynInstPtr,unsigned>>
DependencyGraph<DynInstPtr>::getDependentsOf(PhysRegIndex idx, uint8_t recursiveDepth, vector<PhysRegIndex> &checked) {
    if (recursiveDepth >= maxDependencyRecursion) {
        return vector<pair<DynInstPtr,unsigned>>();
    }
    vector<pair<DynInstPtr,unsigned>> dependentList = vector<pair<DynInstPtr,unsigned>>();
    vector<DynInstPtr> seen = vector<DynInstPtr>();

    DepEntry *curr = &dependGraph[idx];
    while (curr != NULL) {
        if (curr->inst) {
            DynInstPtr inst = curr->inst;
            seen.push_back(inst);
            dependentList.push_back(make_pair(inst,recursiveDepth));
            for (int i = 0; i < inst->numDestRegs(); i++) {
                PhysRegIndex idx = inst->renamedDestRegIdx(i)->flatIndex();
                if (inst->renamedDestRegIdx(i)->isIntPhysReg() || inst->renamedDestRegIdx(i)->isFloatPhysReg()) {
                    if (!count(checked.begin(), checked.end(), idx)) {
                        checked.push_back(idx);
                        vector<pair<DynInstPtr,unsigned>> ret = getDependentsOf(idx, recursiveDepth + 1, checked);
                        for (int i = 0; i < ret.size(); i++) {
                            if (!count(seen.begin(), seen.end(), ret[i].first)) {
                                dependentList.push_back(ret[i]);
                                seen.push_back(ret[i].first);
                            }
                        }
                    }
                }
            }
        }
        curr = curr->next;
    }
    return dependentList;
}

#endif // __CPU_O3_DEP_GRAPH_HH__
