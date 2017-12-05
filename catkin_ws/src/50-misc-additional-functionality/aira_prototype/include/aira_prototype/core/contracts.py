import json
import os

import rospkg
from multiprocessing import Event

from utils import MultihashConverter

__author__ = 'Xomak'

class TransactionException(Exception):
    pass


class Contract:
    def __init__(self, web3, name, address, sender_address=None):
        self.web3 = web3
        self.sender_address = sender_address
        if self.sender_address is None:
            raise ValueError("Can not find account to send transaction from.")
        self.contract = None
        #self.contract_factory = None
        rospack = rospkg.RosPack()
        path_to_package = rospack.get_path('aira_prototype')

        path = os.path.join(path_to_package, "abi", "{}.json".format(name))
        with open(path, "r") as f:
            abi = json.loads(f.read())
            self.contract = web3.eth.contract(
                abi=abi,
                address=address
            )

    def wait_for_transaction(self, transaction):
        feedback_received = Event()

        def callback(block_hash):
            block = self.web3.eth.getBlock(block_hash)
            if transaction in block['transactions']:
                feedback_received.set()

        new_block_filter = self.web3.eth.filter('latest')
        new_block_filter.watch(callback)
        feedback_received.wait()

    def execute_sync(self, contract_func, event):
        transaction_hash = contract_func()
        self.wait_for_transaction(transaction_hash)
        receipt = self.web3.eth.getTransactionReceipt(transaction_hash)
        block_number = receipt['blockNumber']
        if event is not None:
            event_filter = self.contract.pastEvents(event, {'fromBlock': block_number, 'toBlock': 'latest'})
            events = event_filter.get()
            events = [e for e in events if e['transactionHash'] == transaction_hash]
        else:
            events = []
        return {'transactionHash': transaction_hash, 'status': receipt['status'], 'events': events}

    # def at(self, address):
    #     self.contract = self.contract_factory(address)
    #     return self


class RobotLiabilityFactoryContract(Contract):

    REGISTER_GAS_LIMIT = 4000000

    def __init__(self, web3, address, sender_address=None):
        Contract.__init__(self, web3, "RobotLiabilityFactory", address, sender_address)
        self.new_contract_event_filter = self.contract.on('LiabilityRegistered')

    # def at(self, address):
    #     Contract.at(self, address)
    #     self.new_contract_event_filter = self.contract.eventFilter('LiabilityRegistered')
    #     return self

    def create_liability(self, validation_model, confirmation_count, promisee, promisor):
        result = self.execute_sync(lambda: self.contract.transact(
            {"gas": self.REGISTER_GAS_LIMIT, "from": self.sender_address}
        ).createLiability(validation_model, confirmation_count, promisee, promisor), "LiabilityRegistered")
        print(result)
        if result['status'] == 1:
            if len(result['events']) == 1:
                return result['events'][0]['args']['liability']
            else:
                raise TransactionException()
        else:
            raise TransactionException()

    def get_new_contracts(self, promisor_address=None):
        events = self.new_contract_event_filter.get()

        def needed(address):
            print("Got %s" % address)
            return promisor_address is None or address.lower() == promisor_address.lower()

        return [event['args']['liability'] for event in events if needed(event['args']['promisor'])]


class RobotLiabilityContract(Contract):

    def __init__(self, web3, address, sender_address=None):
        Contract.__init__(self, web3, "RobotLiability", address, sender_address)

    def set_objective(self, objective):
        result = self.execute_sync(lambda: self.contract.transact(
            {"from": self.sender_address}
        ).setObjective(MultihashConverter.to_bytes(objective)), None)

    def set_result(self, result):
        result = self.execute_sync(lambda: self.contract.transact(
            {"from": self.sender_address}
        ).setResult(MultihashConverter.to_bytes(result)), None)

    def get_objective(self):
        objective_raw = self.contract.call().objective()
        if len(objective_raw.strip()) > 0:
            return MultihashConverter.to_hash(objective_raw)
        else:
            return None
