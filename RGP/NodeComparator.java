import java.util.Comparator;

public class NodeComparator implements Comparator<Node> {

	@Override
    public int compare(Node node0, Node node1) {
		return Integer.compare(node0.getF(), node1.getF());
    }

}