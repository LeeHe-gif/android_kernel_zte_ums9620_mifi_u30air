/* Started by AICoder, pid:p88eai4c7c28739145d008cde006df9c85d1074a */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// 定义二叉树节点结构体
typedef struct Node {
    int data;
    struct Node *left;
    struct Node *right;
} Node;

// 创建新节点
Node* createNode(int value) {
    Node* newNode = (Node*)malloc(sizeof(Node));
    if (!newNode) {
        printf("Memory error\n");
        return NULL;
    }
    newNode->data = value;
    newNode->left = newNode->right = NULL;
    return newNode;
}

// 插入节点到二叉树
Node* insertNode(Node* node, int value) {
    if (node == NULL) {
        return createNode(value);
    }
    if (value < node->data) {
        node->left = insertNode(node->left, value);
    } else if (value > node->data) {
        node->right = insertNode(node->right, value);
    }
    return node;
}

// 中序遍历二叉树并存储到数组
void inorderTraversal(Node* root, int *array, int *index) {
    if (root != NULL) {
        inorderTraversal(root->left, array, index);
        array[(*index)++] = root->data;
        inorderTraversal(root->right, array, index);
    }
}

void bt_calculate_interface(char *buf) {
    // 检查buf是否为空
    if (buf == NULL) {
        printf("Error: buf is NULL.\n");
        return;
    }

    // 将字符串转换为整数数组
    int nums[3][4];
    char *token = strtok(buf, ",");
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            if (token == NULL) {
                printf("Error: Invalid input format.\n");
                return;
            }
            nums[i][j] = atoi(token);
            token = strtok(NULL, ",");
        }
    }

    // 创建二叉树并插入元素
    Node *root = NULL;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 4; j++) {
            root = insertNode(root, nums[i][j]);
        }
    }

    // 中序遍历二叉树并存储到数组
    int sortedArray[12];
    int index = 0;
    inorderTraversal(root, sortedArray, &index);

    // 打印排序后的数组
    for (int i = 0; i < index; i++) {
        printf("%d ", sortedArray[i]);
    }
    printf("\n");
}

int main() {
    char buf[] = "1,2,3,4,5,6,7,8,9,10,11,12";
    bt_calculate_interface(buf);
    return 0;
}
/* Ended by AICoder, pid:p88eai4c7c28739145d008cde006df9c85d1074a */