import os
from graphviz import Digraph

def generate_swarm_diagram():
    # Создаем ориентированный граф
    dot = Digraph(comment='ROS Swarm Control System')
    
    # --- Настройки внешнего вида (для диплома) ---
    dot.attr(rankdir='LR')       # Слева направо
    dot.attr(splines='ortho')    # Ортогональные линии (прямые углы), выглядит строже
    dot.attr(nodesep='0.6')      # Расстояние между узлами
    dot.attr(ranksep='1.0')      # Расстояние между колонками
    
    # Настройки шрифтов
    dot.attr('node', fontname='Arial', fontsize='12', shape='rect', style='rounded,filled', fillcolor='white')
    dot.attr('edge', fontname='Arial', fontsize='10')

    # --- 1. Центральный узел (Контроллер) ---
    with dot.subgraph(name='cluster_controller') as c:
        c.attr(label='Control Logic', style='dashed', color='grey')
        c.node('Controller', 
               label='Swarm Controller Node\n(swarm_controller.py)', 
               shape='component', 
               fillcolor='lightblue',
               style='filled')

    # --- 2. Узлы MAVROS (для каждого UAV) ---
    uav_list = ['uav0', 'uav1', 'uav2']
    
    for i, uav in enumerate(uav_list):
        # Создаем подгруппу (кластер) для визуального объединения пространства имен
        with dot.subgraph(name=f'cluster_{uav}') as c:
            c.attr(label=f'Namespace: /{uav}', color='black', style='solid')
            
            # Узел MAVROS
            mavros_node = f'mavros_{uav}'
            c.node(mavros_node, 
                   label=f'MAVROS Node\n({uav})', 
                   fillcolor='lightgrey')
            
            # --- Связи (Topics & Services) ---
            
            # Входящие в контроллер (Телеметрия)
            # Используем HTML-подобные метки для красивого форматирования
            telemetry_label = f'<{uav}/mavros/state<br/>{uav}/mavros/local_position/pose>'
            dot.edge(mavros_node, 'Controller', label=telemetry_label, color='blue')
            
            # Исходящие из контроллера (Команды)
            command_label = f'<{uav}/mavros/setpoint_position/local<br/><i>Services: Arming, SetMode</i>>'
            dot.edge('Controller', mavros_node, label=command_label, color='red', style='dashed')

    # --- Сохранение и вывод ---
    output_filename = 'swarm_architecture'
    try:
        # Сохраняет исходник .gv и рендерит .png
        dot.render(output_filename, view=True, format='png')
        print(f"Схема успешно сохранена как {output_filename}.png")
    except Exception as e:
        print("Ошибка при рендеринге. Убедитесь, что Graphviz установлен в системе (PATH).")
        print(f"Детали: {e}")
        # Сохраним хотя бы исходный код графа, если рендер не работает
        with open(output_filename + '.gv', 'w') as f:
            f.write(dot.source)
        print(f"Исходный код графа сохранен в {output_filename}.gv")

if __name__ == '__main__':
    generate_swarm_diagram()