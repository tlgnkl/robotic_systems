"""Sales report generator for Exercise 12 from `home_work_python.md`.

Calculates total revenue, best-selling product by quantity, product with
maximum revenue, and average check per item.
"""

from __future__ import annotations

from collections import Counter

def report_generator() -> None:
    """Generate and print a comprehensive sales report."""
    sales_data = [
        {"product": "Ноутбук", "price": 50_000, "quantity": 3},
        {"product": "Мышь", "price": 1_500, "quantity": 10},
        {"product": "Клавиатура", "price": 3_000, "quantity": 5},
        {"product": "Монитор", "price": 20_000, "quantity": 2},
    ]

    total_revenue = 0
    revenue_by_product = {}
    quantity_counter = Counter()
    total_quantity = 0

    for entry in sales_data:
        product = entry["product"]
        price = entry["price"]
        quantity = entry["quantity"]
        revenue = price * quantity

        total_revenue += revenue
        revenue_by_product[product] = revenue
        quantity_counter[product] += quantity
        total_quantity += quantity

    best_selling_product, best_selling_qty = quantity_counter.most_common(1)[0]
    top_revenue_product = max(revenue_by_product.items(), key=lambda item: item[1])
    average_check = total_revenue / total_quantity if total_quantity else 0

    print("=== ОТЧЕТ О ПРОДАЖАХ ===")
    print(f"Общая выручка: {total_revenue:,.0f} руб.".replace(",", " "))
    print(
        f"Самый продаваемый товар: {best_selling_product} ({best_selling_qty} шт.)"
    )
    print(
        f"Максимальная выручка: {top_revenue_product[0]} "
        f"({top_revenue_product[1]:,} руб.)".replace(",", " ")
    )
    print(f"Средний чек (за единицу): {average_check:,.2f} руб.".replace(",", " "))

if __name__ == "__main__":
    report_generator()
