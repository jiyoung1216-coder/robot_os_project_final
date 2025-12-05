# msg/CookedRamen.msg
ramen_interfaces/RamenOrder order     # 원본 주문 정보
string cook_status                  # 조리 상태
                                    # "perfect" (o)
                                    # "undercooked" (x)
                                    # "overcooked" (x)
                                    # "missing_topping" (x)
                                    # "missing_side" (x)
