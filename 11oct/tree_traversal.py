"""
Различные способы обхода деревьев
"""

from binary_search_tree import BinarySearchTree


class TreeTraversal:
    @staticmethod
    def inorder(node):
        """Центрированный обход (левый-корень-правый)"""
        result = []
        if node:
            result.extend(TreeTraversal.inorder(node.left))
            result.append(node.value)
            result.extend(TreeTraversal.inorder(node.right))
        return result
    
    @staticmethod
    def preorder(node):
        """Прямой обход (корень-левый-правый)"""
        result = []
        if node:
            result.append(node.value)
            result.extend(TreeTraversal.preorder(node.left))
            result.extend(TreeTraversal.preorder(node.right))
        return result
    
    @staticmethod
    def postorder(node):
        """Обратный обход (левый-правый-корень)"""
        result = []
        if node:
            result.extend(TreeTraversal.postorder(node.left))
            result.extend(TreeTraversal.postorder(node.right))
            result.append(node.value)
        return result
    
    @staticmethod
    def level_order(node):
        """Обход в ширину (по уровням)"""
        if not node:
            return []
        
        result = []
        queue = [node]
        
        while queue:
            current = queue.pop(0)
            result.append(current.value)
            
            if current.left:
                queue.append(current.left)
            if current.right:
                queue.append(current.right)
        
        return result


if __name__ == "__main__":
    # Пример использования обходов
    bst = BinarySearchTree()
    for value in [50, 30, 70, 20, 40, 60, 80]:
        bst.insert(value)
    
    print("Дерево:")
    bst.display()
    print("\nОбходы дерева:")
    print("Inorder (симметричный):", TreeTraversal.inorder(bst.root))
    print("Preorder (прямой):", TreeTraversal.preorder(bst.root))
    print("Postorder (обратный):", TreeTraversal.postorder(bst.root))
    print("Level order (по уровням):", TreeTraversal.level_order(bst.root))
